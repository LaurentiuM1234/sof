// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2019 Intel Corporation. All rights reserved.
//
// Author: Tomasz Lauda <tomasz.lauda@linux.intel.com>

#include <sof/audio/component.h>
#include <sof/bit.h>
#include <sof/drivers/interrupt.h>
#include <sof/drivers/timer.h>
#include <sof/lib/alloc.h>
#include <sof/lib/cpu.h>
#include <sof/lib/dma.h>
#include <sof/lib/memory.h>
#include <sof/lib/notifier.h>
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

/* For i.MX, when building SOF with Zephyr, we use wrapper.c,
 * interrupt.c and interrupt-irqsteer.c which causes name
 * collisions.
 * In order to avoid this and make any second level interrupt
 * handling go through interrupt-irqsteer.c define macros to
 * rename the duplicated functions.
 */
#if defined(__ZEPHYR__) && defined(CONFIG_IMX)
#define interrupt_get_irq mux_interrupt_get_irq
#define interrupt_register mux_interrupt_register
#define interrupt_unregister mux_interrupt_unregister
#define interrupt_enable mux_interrupt_enable
#define interrupt_disable mux_interrupt_disable
#endif

#define ZEPHYR_LL_STACK_SIZE 8192

K_KERNEL_STACK_ARRAY_DEFINE(zephyr_dma_domain_stack,
			    CONFIG_CORE_COUNT,
			    ZEPHYR_LL_STACK_SIZE);

struct driving_chan {
	struct dma_chan_data *data; /* driving channel data */
	int irq; /* driving channel IRQ number */
	int core; /* core on which the DMA IRQ was registered */
	struct pipeline_task *ptask; /* driving channel pipeline task */
	bool found; /* true if driving channel has been found */
};

struct zephyr_dma_domain_thread {
	struct k_thread handle;
	struct k_sem sem;
	void (*handler)(void *arg);
	void *arg;
};

struct zephyr_dma_domain {
	struct dma *dma_array; /* array of scheduling DMAs */
	uint32_t dma_count; /* number of scheduling DMAs */
	struct driving_chan driving_chan; /* channel driving the scheduling */

	struct zephyr_dma_domain_thread domain_thread[CONFIG_CORE_COUNT];
};

static void zephyr_dma_domain_irq_handler(void *data)
{
	struct zephyr_dma_domain *zephyr_dma_domain;
	struct driving_chan *driving_chan;
	struct zephyr_dma_domain_thread *dt;
	int core;

	zephyr_dma_domain = data;
	driving_chan = &zephyr_dma_domain->driving_chan;

	for (core = 0; core < CONFIG_CORE_COUNT; core++) {
		dt = zephyr_dma_domain->domain_thread + core;

		if (dt->handler)
			k_sem_give(&dt->sem);
	}

	dma_interrupt_legacy(driving_chan->data, DMA_IRQ_CLEAR);
#ifdef CONFIG_IMX
	dma_interrupt_legacy(driving_chan->data, DMA_IRQ_MASK);
#endif
}

static void zephyr_dma_domain_thread_fn(void *p1, void *p2, void *p3)
{
	struct zephyr_dma_domain *zephyr_dma_domain;
	struct zephyr_dma_domain_thread *dt;
	struct driving_chan *driving_chan;
	int core;

	zephyr_dma_domain = p1;
	core = cpu_get_id();
	dt = zephyr_dma_domain->domain_thread + core;
	driving_chan = &zephyr_dma_domain->driving_chan;

	while (true) {
		k_sem_take(&dt->sem, K_FOREVER);

		dt->handler(dt->arg);
#ifdef CONFIG_IMX
		dma_interrupt_legacy(driving_chan->data, DMA_IRQ_UNMASK);
#endif
	}
}

static int get_driving_chan(struct zephyr_dma_domain *domain,
			    struct pipeline_task *ptask)
{
	struct dma *crt_dma;
	struct dma_chan_data *crt_chan;
	int core, i, j;

	core = cpu_get_id();

	for (i = 0; i < domain->dma_count; i++) {
		crt_dma = domain->dma_array + i;

		for (j = 0; j < crt_dma->plat_data.channels; j++) {
			crt_chan = crt_dma->chan + j;

			/* skip if not set as scheduling source */
			if (!dma_is_scheduling_source(crt_chan))
				continue;

			/* skip if not running */
			if (crt_chan->status != COMP_STATE_ACTIVE)
				continue;

			/* skip if owned by different core */
			if (crt_chan->core != core)
				continue;

			/* if this point is reached then that means we
			 * have found a suitable canditate.
			 *
			 * fill out driving_chan's fields and let caller
			 * know search was successful
			 */
			domain->driving_chan.data = crt_chan;
			domain->driving_chan.core = core;
			domain->driving_chan.ptask = ptask;
			domain->driving_chan.irq = interrupt_get_irq(
			    dma_chan_irq(crt_dma, j),
			    dma_chan_irq_name(crt_dma, j));

			return 0;
		}
	}

	/* if this point is reached then no suitable channel to
	 * drive the scheduling could be found
	 */
	return -EINVAL;
}

static int enable_dma_irq(struct zephyr_dma_domain *domain)
{
	int ret;
	struct driving_chan *driving_chan;

	driving_chan = &domain->driving_chan;

	ret = interrupt_register(driving_chan->irq,
				 zephyr_dma_domain_irq_handler,
				 domain);
	if (ret < 0)
		return ret;

	interrupt_enable(driving_chan->irq, domain);

	dma_interrupt_legacy(driving_chan->data, DMA_IRQ_UNMASK);

	return 0;
}

static int zephyr_dma_domain_register(struct ll_schedule_domain *domain,
					  struct task *task,
					  void (*handler)(void *arg), void *arg)
{
	struct zephyr_dma_domain *zephyr_dma_domain;
	struct pipeline_task *ptask;
	struct zephyr_dma_domain_thread *dt;
	k_tid_t thread;
	k_spinlock_key_t key;
	int core, ret;
	char thread_name[] = "ll_thread0";

	zephyr_dma_domain = ll_sch_domain_get_pdata(domain);
	ptask = pipeline_task_get(task);
	core = cpu_get_id();
	dt = zephyr_dma_domain->domain_thread + core;

	tr_info(&ll_tr, "zephyr_dma_domain_register()");

	/* check if task should be registered */
	if (!ptask->registrable)
		return 0;

	/* check if current core has already called register() */
	if (dt->handler)
		return 0;

	dt->handler = handler;
	dt->arg = arg;

	k_sem_init(&dt->sem, 0, 10);

	thread_name[sizeof(thread_name) - 2] = '0' + core;

	thread = k_thread_create(&dt->handle, zephyr_dma_domain_stack[core],
				ZEPHYR_LL_STACK_SIZE, zephyr_dma_domain_thread_fn,
				zephyr_dma_domain, NULL,
				NULL, -CONFIG_NUM_COOP_PRIORITIES,
				0, K_FOREVER);

	k_thread_cpu_mask_clear(thread);
	k_thread_cpu_mask_enable(thread, core);
	k_thread_name_set(thread, thread_name);

	k_thread_start(thread);

	key = k_spin_lock(&domain->lock);

	/* enable DMA IRQ only if it hasn't been already done
	 *
	 * although the DMA IRQ is registered on a single core
	 * (the first one who calls zephyr_dma_domain_register)
	 * the ISR will give resources to all thread semaphores
	 * (we have 1 thread executing the work per core) so
	 * the work will be executed on all cores
	 */
	if (!zephyr_dma_domain->driving_chan.found) {
		ret = get_driving_chan(zephyr_dma_domain, ptask);
		if (ret < 0) {
			/* this is fine, maybe another core will find
			 * a suitable driving channel
			 */
			k_spin_unlock(&domain->lock, key);

			return 0;
		}

		ret = enable_dma_irq(zephyr_dma_domain);
		if (ret < 0) {
			tr_err(&ll_tr, "failed to enable DMA IRQ on core %d",
			       core);

			k_spin_unlock(&domain->lock, key);

			return ret;
		}

		zephyr_dma_domain->driving_chan.found = true;
	}

	k_spin_unlock(&domain->lock, key);

	return 0;
}

static void disable_dma_irq(struct zephyr_dma_domain *domain)
{
	struct driving_chan *driving_chan = &domain->driving_chan;

	dma_interrupt_legacy(driving_chan->data, DMA_IRQ_MASK);
	interrupt_disable(driving_chan->irq, domain);
	interrupt_unregister(driving_chan->irq, domain);
}

static int zephyr_dma_domain_unregister(struct ll_schedule_domain *domain,
					    struct task *task,
					    uint32_t num_tasks)
{
	struct zephyr_dma_domain *zephyr_dma_domain;
	struct pipeline_task *pipe_task;
	struct driving_chan *driving_chan;
	int core;
	k_spinlock_key_t key;

	tr_info(&ll_tr, "zephyr_dma_domain_unregister()");

	core = cpu_get_id();
	pipe_task = pipeline_task_get(task);
	zephyr_dma_domain = ll_sch_domain_get_pdata(domain);
	driving_chan = &zephyr_dma_domain->driving_chan;

	/* check if task should be unregistered */
	if (num_tasks)
		return 0;

	key = k_spin_lock(&domain->lock);

	if (!driving_chan->found) {
		disable_dma_irq(zephyr_dma_domain);

		driving_chan->found = false;
	}

	zephyr_dma_domain->domain_thread[core].handler = NULL;

	k_spin_unlock(&domain->lock, key);

	k_thread_abort(&zephyr_dma_domain->domain_thread[core].handle);

	return 0;
}

static bool zephyr_dma_domain_is_pending(struct ll_schedule_domain *domain,
					     struct task *task, struct comp_dev **comp)
{
	struct pipeline_task *ptask = pipeline_task_get(task);

	if (!ptask->registrable && task->start > k_uptime_get())
		return false;

	return true;
}

const struct ll_schedule_domain_ops zephyr_dma_domain_ops = {
	.domain_register	= zephyr_dma_domain_register,
	.domain_unregister	= zephyr_dma_domain_unregister,
	.domain_is_pending	= zephyr_dma_domain_is_pending,
};

struct ll_schedule_domain *zephyr_dma_domain_init(struct dma *dma_array,
						  uint32_t dma_count,
						  int clk)
{
	struct ll_schedule_domain *domain;
	struct zephyr_dma_domain *zephyr_dma_domain;

	tr_info(&ll_tr, "zephyr_dma_domain_init(): dma_count %d, clk %d",
		dma_count, clk);

	domain = domain_init(SOF_SCHEDULE_LL_DMA, clk, true,
			     &zephyr_dma_domain_ops);

	zephyr_dma_domain = rzalloc(SOF_MEM_ZONE_SYS_SHARED,
				    0,
				    SOF_MEM_CAPS_RAM,
				    sizeof(*zephyr_dma_domain));

	zephyr_dma_domain->dma_array = dma_array;
	zephyr_dma_domain->dma_count = dma_count;

	ll_sch_domain_set_pdata(domain, zephyr_dma_domain);

	return domain;
}
