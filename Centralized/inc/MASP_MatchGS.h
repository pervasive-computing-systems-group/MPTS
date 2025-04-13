/*
 * MASP_MatchGS.h
 *
 * Created by:	Jonathan Diller
 * On: 			January 15, 2023
 *
 * Description: Algorithm to solve the MAS Problem using matching followed distributed gradient descent.
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

#define DEBUG_MASP_MCHGS	DEBUG || 0


class MASP_MatchGS : public MASPSolver {
public:
	MASP_MatchGS();

	void Solve(MASPInput* input, I_solution* I_final);
	// Gets the number of times the algorithm ran
	int getIterations() {return iterationCount;}

protected:
private:
	/*
	 * Takes cost matrix index and converts it into the corresponding task's index. This assumes
	 * that the entries in the matrix are in order based on minimum required agents per task
	 * (e.g. 1.1 1.2 2.1 2.2 .. x.1 x.2 ..). It will return -1 if the given index is past the
	 * minimum number of agents required over tasks (in x range from example).
	 */
	int get_task(int index, MASPInput* input);

	int iterationCount;
};
