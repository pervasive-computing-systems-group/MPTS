/*
 * MASP_BranchAndMatch.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jul 11, 2024
 *
 * Description: Algorithm to solve the MAS Problem to completion by running our
 * custom branch-and-match algorithm. This BnB approach tries to reduce branching
 * through various techniques.
 */

#pragma once

#include <math.h>
#include <cmath>
#include <algorithm>
#include <vector>

#include "Utilities.h"
#include "MASPSolver.h"
#include "MASP_MatchGS.h"

#define DEBUG_MASP_BNM	DEBUG || 0

// Sort agents by branching factor
//#define SORT_AGENTS		1
// Allow the algorithm to prune branches
//#define PRUNING			1
// Give the algorithm a heuristic start
//#define HEURISTIC_START	1

struct BnMAgent_t {
	int branchFactor;
	int agent_i;
	double average_p;

	BnMAgent_t(int i, int bf) {
		branchFactor = bf;
		agent_i = i;
		average_p = 0;
	}
	BnMAgent_t(const BnMAgent_t& other) {
		branchFactor = other.branchFactor;
		agent_i = other.agent_i;
		average_p = other.average_p;
	}

	bool operator<(const BnMAgent_t& str) const {
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



class MASP_BranchAndMatch : public MASPSolver {
public:
	MASP_BranchAndMatch();

	void Solve(MASPInput* input, I_solution* I_final);

protected:
private:
	void assign_next(MASPInput* input, int i, bool** x_ij, I_solution* I_crnt, std::vector<BnMAgent_t>& agentMap);
	// Determine if this is a valid solution
	bool valid_solution(MASPInput* input, bool** x_ij);
	// Counts the number of recursive states required to iterate through (just FYI)
	long int countRecursiveStates(std::vector<BnMAgent_t>& agents);
	// Takes balanced matching index and converts it into the corresponding agent's index.
	// Returns -1 if this is a phantom
	int get_agent(int i, int N);
	// Takes balanced matching index and converts it into the corresponding task's index
	int get_task(int bal_j, int M, int a_f, std::vector<int>& d_j);
	// Determines if this is a floating task
	bool floating_task(int bal_j, int M, int a_f, std::vector<int>& d_j);

	// Best found solution
	double fGlobalProbSuccess;
	bool** bGlobalX_ij;
	// Performance statistics
	long int nInvalidCount;
	long int nPruningCount;

	// Poisson Binomial helper
	PoissonBinomial m_PoissonB;
};
