/*
 * MASP_EdgeCutting.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jan 10, 2024
 *
 * Description: Edge Cutting algorithm to solve the MAS Problem by starting with
 * the theoretical optimal solution (assign all agents to all task) then repeated
 * removes agents from tasks until the solution is valid.
 */

#pragma once

#include <math.h>
#include <algorithm>
#include <vector>

#include "Utilities.h"
#include "MASPSolver.h"

#define DEBUG_MASP_EC	DEBUG || 1


class MASP_EdgeCutting : public MASPSolver {
public:
	MASP_EdgeCutting();

	void Solve(MASPInput* input, I_solution* I_final);

protected:
private:
	// Returns true if each agent is assigned to at most one task
	bool validSolution(MASPInput* input, bool** x_ij);

	// Poisson Binomial helper
	PoissonBinomial m_PoissonB;
};
