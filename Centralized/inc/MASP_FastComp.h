/*
 * MASP_FastComp.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jan 05, 2024
 *
 * Description: Algorithm to solve the MAS Problem to completion by running
 * our custom branch-and-bound algorithm. This BnB approach tries to reduce
 * branching through various techniques.
 */

#pragma once

#include <math.h>
#include <algorithm>
#include <vector>

#include "Utilities.h"
#include "MASPSolver.h"
#include "MASP_MatchGS.h"

#define DEBUG_MASP_FC	DEBUG || 0

// Sort agents by branching factor
#define SORT_AGENTS		1
// Allow the algorithm to prune branches
#define PRUNING			1
// Give the algorithm a heuristic start
#define HEURISTIC_START	1

struct FastAgent_t {
	int branchFactor;
	int agent_i;
	double average_p;

	FastAgent_t(int i, int bf) {
		branchFactor = bf;
		agent_i = i;
		average_p = 0;
	}
	FastAgent_t(const FastAgent_t& other) {
		branchFactor = other.branchFactor;
		agent_i = other.agent_i;
		average_p = other.average_p;
	}

	bool operator<(const FastAgent_t& str) const {
		if(str.branchFactor < branchFactor) {
			return true;
		}
		else if(str.branchFactor == branchFactor) {
			return str.average_p < average_p;
		}
		else {
			return false;
		}
	}
};



class MASP_FastComp : public MASPSolver {
public:
	MASP_FastComp();

	void Solve(MASPInput* input, I_solution* I_final);

protected:
private:
	void assign_next(MASPInput* input, int i, bool** x_ij, I_solution* I_crnt, std::vector<FastAgent_t>& agentMap);
	// Determine if this is a valid solution
	bool valid_solution(MASPInput* input, bool** x_ij);
	// Counts the number of recursive states required to iterate through (just FYI)
	long int countRecursiveStates(std::vector<FastAgent_t>& agents);

	// Best found solution
	double fGlobalProbSuccess;
	bool** bGlobalX_ij;
	// Performance statistics
	long int nInvalidCount;
	long int nPruningCount;

	// Poisson Binomial helper
	PoissonBinomial m_PoissonB;
};
