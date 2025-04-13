/*
 * MASP_comp.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 3, 2023
 *
 * Description: Algorithm to solve the MAS Problem to completion (i.e. finds
 * the optimal solution through exhaustive search)
 */

#pragma once

#include <math.h>
#include <vector>

#include "Utilities.h"
#include "MASPSolver.h"

#define DEBUG_MASPC		DEBUG || 0


class MASPComp : public MASPSolver {
public:
	MASPComp();

	void Solve(MASPInput* input, I_solution* I_final);

protected:
private:
	void assign_next(MASPInput* input, int i, bool** x_ij, I_solution* I_crnt);
	// Determine if this is a valid solution
	bool valid_solution(MASPInput* input, bool** x_ij);

	// Best found solution
	double fProbSuccess;
	bool** bX_ij;
};
