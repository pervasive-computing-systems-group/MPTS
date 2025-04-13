/*
 * HungGSSolver.h
 *
 * Created by:	Jonathan Diller
 * On: 			January 17, 2024
 *
 * Description: Combination of the Hungarian method + a gradient descent algorithm
 *
 */

#pragma once

#include "defines.h"
#include "Utilities.h"
#include "Node.h"
#include "Solver.h"
#include "PoissonBinomial.h"

#define DEBUG_HUNGGS	DEBUG || 0

#define DELTA			((double*)packet->buffer)[0]
#define T_STAR			((int*)(packet->buffer+9))[0]
#define R_STAR			((int*)(packet->buffer+9))[1]
#define TASK_LIST		((int*)packet->buffer)

class HungGSSolver :  public Node, public Solver {
public:
	HungGSSolver(int nodeID, std::string input_path);
	~HungGSSolver();

protected:
	// Starts the gradient descent algorithm
	void StartAlgo();
	// Runs the gradient descent algorithm
	void RunAlgo(packet_t* packet);

private:
	// minSlack_i = min_j∈F_1 (c_ij + α_j − β_i ) and n_i be the task corresponding to the arg min
	double getMinSlack_i(int* n_i);
	/*
	 * Takes cost matrix index and converts it into the corresponding task's index. This assumes
	 * that the entries in the matrix are in order based on minimum required agents per task
	 * (e.g. 1.1 1.2 2.1 2.2 .. x.1 x.2 ..). It will return -1 if the given index is past the
	 * minimum number of agents required over tasks (in x range from example).
	 */
	int get_task(int index);
	// We turned the problem into a balanced matching problem, so there may not actually be a task j
	double getMtch_q_j(int j);
	/*
	 * After a node is notified to move out of F2, it needs to move the rest of its sub-tree out
	 * of F2 then notify either its next sibling or the former parent. This function performs
	 * the last part of this process.
	 */
	void contMovingF2(packet_t* packet);

	// Algorithm variables. We swapped i with j from the paper.
	double* q_j; // Our cost value
	int M_i;
	double* alpha_j;
	double beta_i;
	double minSlack_i; // Value of minimum slack among task alpha_j and beta_i among tasks in F1
	int n_i;	// Task that corresponds with minSlack_i
	// δ = min_i∈F_2 minSlack_i
	double delta;
	/// Forest membership
	// Is this robot in F2?
	bool F2;
	// Is each task in F1?
	bool* F1_j;
	/// Relationships
	int nFather;
	int nFatherTask;
	int nSon;
	int nOlderBrother;
	int nYungerBrother;

	/// Former Relationships
	int nOldFather;
	int nOldFatherTask;
	int nOldOlderBrother;
	int nOldYungerBrother;

	int nTempFather;
	int nTempSibling;

	// Record where we last assigned ourselves...
	int currentJ;
};
