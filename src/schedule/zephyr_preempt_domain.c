// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright 2022 NXP
//
// Author: Paul Olaru <paul.olaru@nxp.com>
// Author: Laurentiu Mihalcea <laurentiu.mihalcea@nxp.com>

#include <sof/audio/component.h>
#include <sof/bit.h>
#include <sof/drivers/interrupt.h>
#include <sof/drivers/timer.h>
#include <sof/lib/alloc.h>
#include <sof/lib/cpu.h>
#include <sof/lib/dma.h>
#include <sof/platform.h>
#include <sof/schedule/ll_schedule.h>
#include <sof/schedule/ll_schedule_domain.h>
#include <sof/schedule/schedule.h>
#include <sof/schedule/task.h>
#include <ipc/topology.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

LOG_MODULE_DECLARE(ll_schedule, CONFIG_SOF_LOG_LEVEL);

#ifdef CONFIG_IMX
#define interrupt_get_irq mux_interrupt_get_irq
#define interrupt_register mux_interrupt_register
#define interrupt_unregister mux_interrupt_unregister
#define interrupt_enable mux_interrupt_enable
#define interrupt_disable mux_interrupt_disable
#endif

#define SEM_LIMIT 1
#define ZEPHYR_LL_STACK_SIZE 8192

K_KERNEL_STACK_ARRAY_DEFINE(zephyr_preempt_domain_stack,
			    CONFIG_CORE_COUNT,
			    ZEPHYR_LL_STACK_SIZE);

struct zephyr_preempt_domain_data {
	int irq; /* IRQ associated with channel */
	struct dma_chan_data *channel; /* channel data */
	bool enabled_irq; /* true if DMA IRQ was already enabled */
	struct pipeline_task *pipe_task;
	struct zephyr_preempt_domain_thread *dt;
};

struct zephyr_preempt_domain_thread {
	struct k_thread ll_thread; /* thread handle */
	struct k_sem sem; /* used to signal when work should be done */
	void (*handler)(void *arg); /* work to be done */
	void *arg; /* data used by work function */
	bool started; /* true if the thread was started */
};

struct zephyr_preempt_domain {
	struct dma *dma_array; /* array of scheduling DMAs */
	uint32_t num_dma; /* number of scheduling DMAs */

	struct zephyr_preempt_domain_data data[PLATFORM_NUM_DMACS][PLATFORM_MAX_DMA_CHAN];
	/* array of Zephyr threads - one for each core */
	struct zephyr_preempt_domain_thread domain_thread[CONFIG_CORE_COUNT];

	struct k_timer timer; /* used whenever the DMA IRQs get stopped */
	bool timer_init; /* true if timer was initialized */

	struct ll_schedule_domain *domain;

	/* true if timer should give sem resources to a Zephyr thread
	 * running on some core
	 */
	bool core_timer_enable[CONFIG_CORE_COUNT];
};

static int zephyr_preempt_domain_register(struct ll_schedule_domain *domain,
					  struct task *task,
					  void (*handler)(void *arg),
					  void *arg);
static int zephyr_preempt_domain_unregister(struct ll_schedule_domain *domain,
					    struct task *task,
					    uint32_t num_tasks);
static bool zephyr_preempt_domain_is_pending(struct ll_schedule_domain *domain,
					     struct task *task,
					     struct comp_dev **comp);
static void zephyr_preempt_domain_task_cancel(struct ll_schedule_domain *domain,
					      struct task *task,
					      uint32_t num_tasks);

static const struct ll_schedule_domain_ops zephyr_preempt_domain_ops = {
	.domain_register	= zephyr_preempt_domain_register,
	.domain_unregister	= zephyr_preempt_domain_unregister,
	.domain_is_pending	= zephyr_preempt_domain_is_pending,
	.domain_task_cancel	= zephyr_preempt_domain_task_cancel
};


struct ll_schedule_domain *zephyr_preempt_domain_init(struct dma *dma_array,
						      uint32_t num_dma,
						      int clk)
{
	struct ll_schedule_domain *domain;
	struct zephyr_preempt_domain *zephyr_preempt_domain;

	/* initialize domain */
	domain = domain_init(SOF_SCHEDULE_LL_DMA,
			     clk,
			     true,
			     &zephyr_preempt_domain_ops);

	/* initialize domain pdata */
	zephyr_preempt_domain = rzalloc(SOF_MEM_ZONE_SYS_SHARED,
					0,
					SOF_MEM_CAPS_RAM,
					sizeof(*zephyr_preempt_domain));

	zephyr_preempt_domain->dma_array = dma_array;
	zephyr_preempt_domain->num_dma = num_dma;
	zephyr_preempt_domain->domain = domain;

	/* set pdata */
	ll_sch_domain_set_pdata(domain, zephyr_preempt_domain);

	return domain;
}

static void zephyr_preempt_domain_thread_fn(void *p1, void *p2, void *p3)
{
	struct zephyr_preempt_domain_thread *dt = p1;

	while (true) {
		/* wait for DMA IRQ */
		k_sem_take(&dt->sem, K_FOREVER);

		/* do work */
		dt->handler(dt->arg);
	}
}

static void dma_irq_handler(void *data)
{
	struct zephyr_preempt_domain_data *zephyr_preempt_domain_data;
	struct dma_chan_data *channel;
	struct k_sem *sem;
	int channel_index, irq;

	zephyr_preempt_domain_data = data;
	channel = zephyr_preempt_domain_data->channel;
	channel_index = channel->index;
	sem = &zephyr_preempt_domain_data->dt->sem;
	irq = zephyr_preempt_domain_data->irq;

	/* give resources to thread semaphore */
	k_sem_give(sem);

	/* clear IRQ */
	dma_interrupt_legacy(channel, DMA_IRQ_CLEAR);
	interrupt_clear_mask(irq, BIT(channel_index));
}

static int enable_dma_irq(struct zephyr_preempt_domain_data *data)
{
	int ret;

	dma_interrupt_legacy(data->channel, DMA_IRQ_CLEAR);

	ret = interrupt_register(data->irq, dma_irq_handler, data);
	if (ret < 0)
		return ret;

	interrupt_enable(data->irq, data);

	interrupt_clear_mask(data->irq, BIT(data->channel->index));

	dma_interrupt_legacy(data->channel, DMA_IRQ_UNMASK);

	return 0;
}

static int register_dma_irq(struct zephyr_preempt_domain *domain,
			    struct zephyr_preempt_domain_data **data,
			    struct zephyr_preempt_domain_thread *dt,
			    struct pipeline_task *pipe_task,
			    int core)
{
	struct dma *crt_dma;
	struct dma_chan_data *crt_chan;
	struct zephyr_preempt_domain_data *crt_data;
	int i, j, irq, ret;

	/* register the DMA IRQ only for PPL tasks marked as "registrable"
	 *
	 * this is done because, in case of mixer topologies there's
	 * multiple PPLs having the same scheduling component so there's
	 * no need to go through this function for all of those PPL
	 * tasks - only the PPL task containing the scheduling component
	 * will do the registering
	 *
	 */
	if (!pipe_task->registrable)
		return 0;

	/* iterate through all available channels in order to find a
	 * suitable channel for which the DMA IRQs will be enabled.
	 */
	for (i = 0; i < domain->num_dma; i++) {
		crt_dma = domain->dma_array + i;

		for (j = 0; j < crt_dma->plat_data.channels; j++) {
			crt_chan = crt_dma->chan + j;
			crt_data = &domain->data[i][j];

			/* skip if channel is not a scheduling source */
			if (!dma_is_scheduling_source(crt_chan))
				continue;

			/* skip if channel is not active */
			if (crt_chan->status != COMP_STATE_ACTIVE)
				continue;

			/* skip if channel not owned by current core */
			if (core != crt_chan->core)
				continue;

			/* skip if IRQ was already registered */
			if (crt_data->enabled_irq)
				continue;

			/* get IRQ number for current channel */
			irq = interrupt_get_irq(dma_chan_irq(crt_dma, j),
						dma_chan_irq_name(crt_dma, j));

			crt_data->irq = irq;
			crt_data->channel = crt_chan;
			crt_data->dt = dt;
			crt_data->pipe_task = pipe_task;

			if (dt->started) {
				/* if the Zephyr thread was started, we
				 * can safely enable the DMA IRQs
				 */
				ret = enable_dma_irq(crt_data);
				if (ret < 0)
					return ret;

				crt_data->enabled_irq = true;
			}

			/* let caller know we have found a channel */
			*data = crt_data;

			return 0;
		}
	}

	/* if this point is reached then that means we weren't able to
	 * find a suitable channel, let caller know
	 */
	return -EINVAL;
}

static void zephyr_preempt_domain_timer_fn(struct k_timer *timer)
{
	struct zephyr_preempt_domain *zephyr_preempt_domain;
	int core;
	struct zephyr_preempt_domain_thread *dt;

	zephyr_preempt_domain = k_timer_user_data_get(timer);

	for (core = 0; core < CONFIG_CORE_COUNT; core++) {
		dt = zephyr_preempt_domain->domain_thread + core;

		if (zephyr_preempt_domain->core_timer_enable[core])
			k_sem_give(&dt->sem);
	}
}

static void timer_init(struct zephyr_preempt_domain *domain)
{
	k_timeout_t start = {0};

	k_timer_init(&domain->timer, zephyr_preempt_domain_timer_fn, NULL);
	k_timer_user_data_set(&domain->timer, domain);
	k_timer_start(&domain->timer, start, K_USEC(LL_TIMER_PERIOD_US));

	domain->timer_init = true;
}

static int zephyr_preempt_domain_register(struct ll_schedule_domain *domain,
					  struct task *task,
					  void (*handler)(void *arg),
					  void *arg)
{
	struct zephyr_preempt_domain *zephyr_preempt_domain;
	struct zephyr_preempt_domain_thread *dt;
	struct pipeline_task *pipe_task;
	struct zephyr_preempt_domain_data *data;
	k_tid_t thread;
	int core, ret;
	k_spinlock_key_t key;
	char thread_name[] = "ll_thread0";

	zephyr_preempt_domain = ll_sch_get_pdata(domain);
	core = cpu_get_id();
	data = NULL;
	dt = zephyr_preempt_domain->domain_thread + core;
	pipe_task = pipeline_task_get(task);

	tr_info(&ll_tr, "zephyr_preempt_domain_register()");

	key = k_spin_lock(&domain->lock);
	if (!zephyr_preempt_domain->timer_init)
		timer_init(zephyr_preempt_domain);
	k_spin_unlock(&domain->lock, key);

	/* the DMA IRQ has to be reigstered before the Zephyr thread is
	 * started.
	 *
	 * this is done because we can have multiple DMA IRQs giving
	 * resources to the Zephyr thread semaphore on the same core
	 */
	ret = register_dma_irq(zephyr_preempt_domain,
			       &data,
			       dt,
			       pipe_task,
			       core);

	/* TODO: maybe print an error message here */
	if (ret < 0) {
		tr_err(&ll_tr,
		       "failed to register DMA IRQ for pipe task %p on core %d",
		       pipe_task, core);
		return ret;
	}

	tr_info(&ll_tr, "zephyr_preempt_domain_register(): stopping timer on core %d", core);

	/* disable timer for current core since we should be receiving
	 * DMA IRQs
	 */
	zephyr_preempt_domain->core_timer_enable[core] = false;

	/* skip if Zephyr thread was already started on this core */
	if (dt->handler)
		return 0;

	dt->handler = handler;
	dt->arg = arg;

	/* prepare work semaphore */
	k_sem_init(&dt->sem, 0, SEM_LIMIT);

	thread_name[sizeof(thread_name) - 2] = '0' + core;

	/* create Zephyr thread */
	thread = k_thread_create(&dt->ll_thread,
				 zephyr_preempt_domain_stack[core],
				 ZEPHYR_LL_STACK_SIZE,
				 zephyr_preempt_domain_thread_fn,
				 dt,
				 NULL,
				 NULL,
				 -CONFIG_NUM_COOP_PRIORITIES,
				 0,
				 K_FOREVER);

	k_thread_cpu_mask_clear(thread);
	k_thread_cpu_mask_enable(thread, core);
	k_thread_name_set(thread, thread_name);

	k_thread_start(thread);

	dt->started = true;

	/* TODO: maybe remove the second condition */
	if (data && !data->enabled_irq) {
		/* enable the DMA IRQ since register_dma_irq wasn't able
		 * to do so because of the fact that the Zephyr thread
		 * hadn't started at that point
		 */
		ret = enable_dma_irq(data);
		if (ret < 0) {
			tr_err(&ll_tr,
			       "failed to enable DMA IRQ for pipe task %p on core %d",
			       pipe_task,
			       core);
			return ret;
		}

		data->enabled_irq = true;

		return 0;
	}

	/* if this point is reached then that means we weren't able to
	 * enable DMA IRQ either because data was NULL or the IRQ was
	 * already enabled even though the Zephyr thread wasn't started
	 */

	tr_err(&ll_tr, "failed to register pipeline task %p on core %d",
	       pipe_task,
	       core);

	return -EINVAL;
}

static void disable_dma_irq(struct zephyr_preempt_domain_data *data)
{
	dma_interrupt_legacy(data->channel, DMA_IRQ_MASK);
	dma_interrupt_legacy(data->channel, DMA_IRQ_CLEAR);
	interrupt_clear_mask(data->irq, BIT(data->channel->index));
	interrupt_disable(data->irq, data);
	interrupt_unregister(data->irq, data);
}

static int unregister_dma_irq(struct zephyr_preempt_domain *domain,
			      struct pipeline_task *pipe_task,
			      int core)
{
	struct zephyr_preempt_domain_data *crt_data;
	struct dma *crt_dma;
	int i, j;
	/* unregister the DMA IRQ only for PPL tasks marked as "registrable"
	 *
	 * this is done because, in case of mixer topologies there's
	 * multiple PPLs having the same scheduling component so there's
	 * no need to go through this function for all of those PPL
	 * tasks - only the PPL task containing the scheduling component
	 * will do the unregistering
	 *
	 */
	if (!pipe_task->registrable)
		return 0;

	for (i = 0; i < domain->num_dma; i++) {
		crt_dma = domain->dma_array + i;

		for (j = 0; j < crt_dma->plat_data.channels; j++) {
			crt_data = &domain->data[i][j];

			/* skip if DMA IRQ wasn't enabled */
			if (!crt_data->enabled_irq)
				continue;

			/* skip if channel not owned by current core */
			if (crt_data->channel->core != core)
				continue;

			/* skip if channel is still active */
			if (crt_data->channel->status == COMP_STATE_ACTIVE)
				continue;

			disable_dma_irq(crt_data);

			crt_data->enabled_irq = false;

			return 0;
		}
	}

	/* if this point is reached then something went wrong */
	return -EINVAL;
}

static int zephyr_preempt_domain_unregister(struct ll_schedule_domain *domain,
					    struct task *task,
					    uint32_t num_tasks)
{
	struct zephyr_preempt_domain *zephyr_preempt_domain;
	struct zephyr_preempt_domain_thread *dt;
	struct pipeline_task *pipe_task;
	int ret, core;

	zephyr_preempt_domain = ll_sch_get_pdata(domain);
	pipe_task = pipeline_task_get(task);
	core = cpu_get_id();
	dt = zephyr_preempt_domain->domain_thread + core;

	tr_info(&ll_tr, "zephyr_preempt_domain_unregister()");

	ret = unregister_dma_irq(zephyr_preempt_domain, pipe_task, core);
	if (ret < 0) {
		tr_err(&ll_tr, "failed to unregister DMA IRQ for pipe task %p on core %d",
		       pipe_task,
		       core);
		return ret;
	}

	return 0;
}

static bool zephyr_preempt_domain_is_pending(struct ll_schedule_domain *domain,
					     struct task *task,
					     struct comp_dev **comp)
{
	/* TODO: implementation goes here */
	return true;
}

static void zephyr_preempt_domain_task_cancel(struct ll_schedule_domain *domain,
					      struct task *task,
					      uint32_t num_tasks)
{
	struct zephyr_preempt_domain *zephyr_preempt_domain;
	struct zephyr_preempt_domain_thread *dt;
	int core;

	zephyr_preempt_domain = ll_sch_get_pdata(domain);
	core = cpu_get_id();
	dt = zephyr_preempt_domain->domain_thread + core;

	if (!num_tasks) {
		/* there are no more tasks, timer should start giving
		 * resources to thread running on current core
		 */
		tr_info(&ll_tr, "starting timer on core %d", core);
		zephyr_preempt_domain->core_timer_enable[core] = true;
	}
}
