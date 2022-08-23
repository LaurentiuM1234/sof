/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2022 Intel Corporation. All rights reserved.
 */

/* Created 01-Sep-2022 13:41:14 with script ref_matrix.m v1.9-rc1-1702-gba40860ea */

#define MEL_FILTERBANK_32_TEST2_FFT_SIZE  512
#define MEL_FILTERBANK_32_TEST2_NUM_MEL_BINS  23
#define MEL_FILTERBANK_32_TEST2_NORM_SLANEY  1
#define MEL_FILTERBANK_32_TEST2_MEL_LOG  MEL_LOG

static const int32_t mel_filterbank_32_test2_real[257] = {
	-1553549903,  1519102947,  1971016264,  1952311684,  1502019509, -2135546249,
	  334435081,   669165406,  1997940956, -2087199083,   340140695,  -599719224,
	 -929749784,  1406042526,  -601698859,  1690318057, -1245361691,    14178979,
	 -782867292,   561948993,  -268156857, -1779275050, -1469590716,  2136227042,
	-1051138938,   377177034,  1393457559,  1811805671, -1047327966, -1825997025,
	 1699009346,  -735813687,  -560420644,  1589653182,  1243835006,   592772642,
	 1329146717,  -168503675, -1799787938,   -92096515, -1192632592,   165336535,
	-1284269327,   319393672,   -66833719,  -946695591,  1875495838,  1913540533,
	  870103559,  1012912183,  -813466006,  -712422620,  -732721697, -1513947605,
	-2126885359,   174281503,   -99344256,   693033011,   289849813, -1812611662,
	 1947341544, -1572693848,   -79871082, -1686517696,  -600151820,   270126309,
	 1198773610,   249242746,  1997609326, -1957076810,   750465836, -1967660588,
	 -571549042,    63649055, -1620349479, -1310030477,  1808206363,    89715994,
	  -42192626,  -433912984,  -784537962, -1970410580,   -66668017,   207097229,
	 2099570554,   819708231,  -202921532, -1266941098,   621847351,   422675047,
	 -672235962,  1346150038,   771406375,  1284327990,  -639744697, -1794703700,
	-1973103045,   426408249,  1046976833,  -674381245,  -793076628,  1912376516,
	 1532247715,   -55335413,  1485343965, -1521513513,  1034203470, -1606025163,
	 1622148690,  -459612046,  1941338917,  -490951658,  1421055500,  -589857307,
	 2017263151,   -90863634, -1838427593,  1655020118,  2042412661,   505631122,
	 -588097720,   439176050,  1288704036,   177625976,   627333801,  -392380224,
	-1057924595, -1461152929, -2053033525,   500059969,  -431662173, -1763036449,
	 1432458133,   210881116, -1283298584,  1471939131,  1306846831,  1574504657,
	 -448418718,  1957724094,  1281495437,   995638346,   -79112065,  1460552221,
	-1255773318, -1617308697, -2100870140,  -625847386,   877332297,   442412667,
	 -875655632,  1230707806,  1479507606,  1656880101, -1146192518,  1938963852,
	 -511892005,   801005290, -1845956654,  -918561937,    56587706,  1781921405,
	-1811230777,  1885536047, -1209288380,  -763967730,  1419786033,  -586521120,
	 1622137653, -1536170560,  1869660572,  1791530621, -1583513331,  -997349888,
	-2012168178,  1030013071,  -531920060,  1518027596,   306214102,  -452781269,
	  -76062500, -2127021243,  -522120868,  -427853140,  -625692431,  -871753914,
	 2088486740,  1338997820,  1564882758,  -610779824,    51599573, -1408547901,
	-1436368370,   564767430,  -307855484,   134259371,   508866345,  1114738976,
	 1047675157, -1769750378, -2128483667,    35760449,   -35482204,  -293246337,
	 1691081636, -1182064441, -1251767603, -1821327210, -1586206302,  -232560774,
	 -894265574,  -784082189,  2058903507,   -63857488,  1269757727,   810849511,
	-2070842620,  -117144657,  1098731954,  1251440440,  -742729118, -1390633779,
	-1827605188,   -99423018,  -960160889,   512132057,  1183041220,  -759693445,
	 -988507203,   175925735,   -71774103,   412284802, -1922116143,  1332438951,
	  671866485,  1210253790,   755787757,   478210679,  -112446287,  1949037762,
	 1781663156,  2070734356, -1222017212, -1667906152,  1042401609,  -548665948,
	 2124688009, -1752166648,   349113897,   382157448, -1696822425,  1172848447,
	-1007178649,  -300502759,   669668672,    -6120818, -1133256620,
};

static const int32_t mel_filterbank_32_test2_imag[257] = {
	 2022932185, -1406792770,  1566976099,  1753624708,  1700073129, -1375555002,
	-1821937317,  1000316580, -1612845745, -1444867686,  2088252511, -1500055717,
	 1672500327, -1367435631,  1953579645, -1326518855,    63574235, -2077077324,
	-1674272650,  1415626803,   832889384,  1150459609, -1978078252, -1078045701,
	-1520909025,  -783380774,  -455361749,   128837024,  2066251614, -1429566955,
	-1444139055,  1239540509,    -1722798,  -898803545, -1063555116,  1678175892,
	-1375779419, -2014656908,  1514440207,   712628328,  -805450260,  1656867906,
	 -177078940,   133250594,  -626033481,  1434262370, -1664231819, -1560652944,
	  547365856,   297120813,  1651826879,  -918462546,  1138955519,   409661681,
	 -419367715,  1996770958, -1416714669, -1494586014, -1010779488, -1245515205,
	  988115070,    -9062371,  -372384341,  -886312425, -1974664006, -2123987658,
	  857572255, -1988097401,   531919045,  -229126748, -1638913888,  1735955334,
	 1866112471,  1565238963,  1678707651,  1274123016,  -964164219,   384906508,
	-1936764885,  2018245458,  -147168070,   255018081,  -748474129,   289125392,
	-1091223352, -1491211434, -2066968279,   639471025, -1019531717,    22542264,
	-1792252275,   687066662,  -418959638,  1819293096,   955901488,   310926833,
	  873679284,  1585728339,  -558836273,  -795065626,  -258074231,   943119483,
	  967115019,  1861225771,   680147165, -1931094132,    40446209, -1993649254,
	 -253459156,  1572158301, -1961148205,  -351676454,   234403801,   875096849,
	 -597332084, -1379218468,  1664826405,  -428789343,  2122081012,  1448374339,
	-1723515648, -2083169303,   -41022415,  1815564743,  2118173113,    42017274,
	 -774784352,  1346458836, -1888491234,  2094117390,  1899356514,   229845899,
	  614801967,  1517893772, -1091783149,  1236803895, -1022539339,   807156546,
	  744377231, -1593839411, -1700167272,  -365683655,  1487897182, -1358415330,
	-1962987875,  1919957695,   591262933,  1166931725,   550748162,  1067480207,
	-1998124059,  -606144255,  1180196286,  1864472221,   936848849,   -97942119,
	  379812839, -1889523183,  1990378861,   137158121, -1004198040, -1313478573,
	  -40292123,  1853921188,   687385076, -1289468322,  1366187604, -2033642941,
	   -9922621,  2069678009, -1501937673,  -726402444,  -158623961,  -430670307,
	 1226531165,  -929626641, -2069536254,  1115240213,  -125245517,  1066545490,
	-1248112123,  1911472599,  -637911170,   514244132,   972026997, -1825773878,
	 1337521057, -2114769914,   857915594, -2007583405,   401357491,  1958613210,
	-1265771463,  1546457345,  -845081108,  -912335506,   545899273, -2069650054,
	 1563264325,   322811539,   522142111,  1616357580,   534163593, -1585412227,
	-1556057448,    15378349,   414595713, -1640257820,  1185143276,  -987462108,
	  797782911,  2070063410,  1416132973, -1414259155,  1218593133,  -199454865,
	 -519836010, -1579339806, -2126456109, -1199309429,   148062325,  1316890682,
	-1995024242, -1820536488,  -796775965,    65787848,   297134390,  1437847782,
	   32513752, -2028832706,  -659162650,   679805548,  -808167402,   674264076,
	  665105258,   272564597,   806466739,  1537685956,  -399518310,   342426583,
	-1233245014,  -292727843,  -144628169,  1326810572,  1343469814, -1723094008,
	 2018636797, -1903027505,  1925682521,  1334934547,  -753404568,  -196996733,
	 -373469701,  1768468716,  1918639587,  1080474916,  1631324172,
};

static const int16_t mel_filterbank_32_test2_mel_log[23] = {
	  -451,   -432,   -466,   -473,   -492,   -467,   -468,   -498,   -512,   -510,
	  -510,   -491,   -464,   -492,   -520,   -505,   -475,   -475,   -480,   -478,
	  -480,   -494,   -499,
};
