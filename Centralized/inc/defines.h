/*
 * defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			May 25, 2023
 *
 * Description: Global project defines.
 */

#pragma once

#define DEBUG			0
#define SANITY_PRINT	0

#define EPSILON			0.000001
#define INF				1000000000000
#define PI				3.14159265

enum {
	e_Algo_MASP_COMP = 0,
	e_Algo_MASP_FAST_COMP = 1,
	e_Algo_MASP_TMTCH = 2,
	e_Algo_MASP_BMTCH = 3,
	e_Algo_MASP_MTCHACT = 4,
	e_Algo_MASP_EDGCUT = 5,
	e_Algo_MASP_GRADSEARCH = 6,
	e_Algo_MASP_MTCHGS = 7,
	e_Algo_MASP_MIN_DIST = 8,
	e_Algo_MASP_LOGMATCH = 9,
	e_Algo_MASP_SWAP = 10,
	e_Algo_MASP_BNM = 11,
	e_Algo_MASP_BNB = 12
};
