/*
 * GradientSolver.h
 *
 * Created by:	Jonathan Diller
 * On: 			January 14, 2024
 *
 * Description: Gradient solver, where each agent only makes a move that improves the larger solution.
 *
 */

#pragma once

#include "defines.h"
#include "Utilities.h"
#include "Node.h"
#include "Solver.h"
#include "PoissonBinomial.h"

#define DEBUG_GS		DEBUG || 0


class GradientSolver :  public Node, public Solver {
public:
	GradientSolver(int nodeID, std::string input_path);
	~GradientSolver();

protected:
	// Starts the gradient descent algorithm
	void StartAlgo();
	// Runs the gradient descent algorithm
	void RunAlgo(packet_t* packet);

private:
	// Record where we last assigned ourselves...
	int currentJ;
};
