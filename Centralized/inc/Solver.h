/*
 * Solver.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 7, 2023
 *
 * Description: Parent class for all solvers
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>
#include <complex>

#include "I_solution.h"
#include "MASPInput.h"
#include "PoissonBinomial.h"
#include "Utilities.h"

#define DEBUG_SOLVER	0 || DEBUG

class Solver {
public:
	Solver();
	virtual ~Solver();

	virtual void Solve(MASPInput* input, I_solution* I_final) = 0;

	/*
	 * Determines the probability of mission success given the input problem
	 * and assignment I_sol. Assumes that I_sol contains a valid solution.
	 */
	double BenchmarkRec(MASPInput* input, bool** x_ij);
	/*
	 * Determines the probability of mission success given the input problem
	 * and assignment I_sol. Assumes that I_sol contains a valid solution.
	 */
	double BenchmarkCF(MASPInput* input, bool** x_ij);
protected:
	bool wholeNumber(double);
	// Calculates P_j for the agents in I_j
	double P_j(MASPInput* input, std::vector<int>& I_j, int j);
	// Calculates nCk
	int nChoosek(int n, int k);
	// Calculate S_l
	double S_l(MASPInput* input, std::vector<int>& I_j, int j, int l);
	// Compiles all combinations of length reqLen in list a
	void combination(int* a, int reqLen, int start, int currLen, bool* check, int len, std::vector<std::vector<int>>& combos);

	PoissonBinomial m_poissonBinomial;
};
