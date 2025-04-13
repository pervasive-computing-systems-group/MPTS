/*
 * MASP_MatchAct.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 8, 2023
 *
 * Description: Algorithm to solve the MAS Problem using matching followed blind, fair auction
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

#define DEBUG_MASP_MCHAC	DEBUG || 0


class MASP_MatchAct : public MASPSolver {
public:
	MASP_MatchAct();

	void Solve(MASPInput* input, I_solution* I_final);

protected:
private:
	/*
	 * Takes cost matrix index and converts it into the corresponding task's index. This assumes
	 * that the entries in the matrix are in order based on minimum required agents per task
	 * (e.g. 1.1 1.2 2.1 2.2 .. x.1 x.2 ..). It will return -1 if the given index is past the
	 * minimum number of agents required over tasks (in x range from example).
	 */
	int get_task(int index, MASPInput* input);
	// Runs Monti Carlo Simulation to determine the probability that the agents in I_j complete task j
	double monteSim(MASPInput* input, std::vector<int>& I_j, int j, I_solution* I_final);
};
