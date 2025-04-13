/*
 * MASP_GradientSearch.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jan 14, 2024
 *
 * Description:
 */

#pragma once

#include <tuple>
#include <queue>

#include "Utilities.h"
#include "MASPSolver.h"

#define DEBUG_MASP_GS	DEBUG || 0


class MASP_GradientSearch : public MASPSolver {
public:
	MASP_GradientSearch();

	void Solve(MASPInput* input, I_solution* I_final);
	// Gets the number of times the algorithm ran
	int getIterations() {return iterationCount;}

protected:
private:
	int iterationCount;
};
