/*
 * Solver.h
 *
 * Created by:	Jonathan Diller
 * On: 			January 17, 2024
 *
 * Description: General solver class
 *
 */

#pragma once

#include <sstream>
#include <fstream>
#include <chrono>
#include <tuple>
#include <queue>

#include "defines.h"
#include "Utilities.h"
#include "Node.h"
#include "PoissonBinomial.h"

#define DEBUG_SOLVER	DEBUG || 0

struct agent_tp {
	int task;
	double probability;
};

#define NEIGHBOR		(mNodeId+1)%N
#define X_I(i)			((agent_tp*)packet->buffer)[i]
//#define X_IJ(i,j)		((double*)packet->buffer)[i*N+j]
#define GET_IJ(i,j)		(i*N+j)
#define DELTA			((double*)packet->buffer)[0]


class Solver {
public:
	Solver(int nodeID, std::string input_path);
	virtual ~Solver();

protected:
	// Get next line from input an input file, ignores lines that start with '#'
	bool getNextLine(std::ifstream* file, std::string* line);
	// Determines if i is capable of performing j
	bool iCanDoj(int j);
	/*
	 * Determines the probability of mission success given the solution assignment using
	 * the agent table. This should ignore drones assigned to task -1.
	 */
	double BenchmarkCF(agent_tp* agent_table);
	// Returns the probability that this node performs j
	double get_p_j(int j);

	// Number of agents
	int N;
	// Number of tasks
	int M;
	// Number of capabilities
	int E;
	// This agent's capabilities (size: 1xE)
	int* c_k;
	// Capability requirements per task (size: MxE)
	int** r_jk;
	// Minimum number of agents to complete task j (size:M)
	int* d_j;

	// Note the name/location of the input file (helps track results)
	std::string sInput;

	PoissonBinomial m_poissonBinomial;
	int algorithm;
	std::chrono::system_clock::time_point startTime;
private:
	// Probability that this agent can complete task j (size: 1xM)
	double* p_j;
};
