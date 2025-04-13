/*
 * OnlineSolver.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 3, 2023
 *
 * Description: Parent class for MASP-type solvers.
 */

#pragma once

#include <vector>
#include <math.h>
#include <limits>

#include "Solver.h"
#include "Hungarian.h"

#define DEBUG_MASPSOLV	DEBUG || 0


class MASPSolver : public Solver {
public:
	MASPSolver();
	virtual ~MASPSolver();

protected:
	// Hungarian Algorithm solver
	HungarianAlgorithm mHungAlgo;

private:

	// Sets a = min(a, b), returns true if b < a
	bool ckmin(double* a, double b);
};
