/*
 * MASP_Swap.h
 *
 * Created by:	Will Byers
 * On: 			June 18, 2024
 *
 * Description: Algorithm to solve the MAS Problem by turning problem into balanced matching
 */

#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <limits>
#include <list>
#include <time.h>

#include "Utilities.h"
#include "MASPSolver.h"

#define DEBUG_MASP_SWAP		DEBUG || 1


class MASP_Swap : public MASPSolver {
public:
	MASP_Swap();

	void Solve(MASPInput* input, I_solution* I_final);

protected:
private:
	// Number of required agents
	int a_r;
	// Number of floating agents
	int a_f;
	// Number of participants for SMP
	int np;

	// Takes balanced matching index and converts it into the corresponding task's index
	int get_task(int j, MASPInput* input);
	// Takes balanced matching index and converts it into the corresponding agent's index.
	// Returns -1 if this is a phantom
	int get_agent(int i, MASPInput* input);
	// Determines if this is a floating task
	bool floating_task(int j, MASPInput* input);
	int resolve_task(int jj, MASPInput* input);
	void single_swap(I_solution* I_crnt, MASPInput* input, I_solution& outputSoln);
	void two_swap(I_solution* I_crnt, MASPInput* input, I_solution& outputSoln);
	void three_swap(I_solution* I_crnt, MASPInput* input, I_solution& outputSoln);
	void four_swap(I_solution* I_crnt, MASPInput* input, I_solution& outputSoln);

};
