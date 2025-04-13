/*
 * MASInput.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 3, 2023
 *
 * Description: MAS problem input class
 *
 */

#pragma once

#include <vector>

#include "Input.h"
#include "PoissonBinomial.h"


#define DEBUG_MASPINPUT	DEBUG || 0

class MASPInput : public Input {
public:
	/*
	 * Sets:
	 * i in I, set of agents, |I| = N
	 * j in J, set of tasks, |J| = M
	 * k in K, set of capabilities, |K| = E
	 *
	 * Parameters
	 * c_ik[N][E] - capabilities per agent
	 * r_jk[M][E] - capabilities requirement per task
	 * p_ij[N][M] - probability agent i can complete task j
	 * d_j[M]     - Minimum number of agents required to complete task j
	 */

	MASPInput(std::string input_path);
	virtual ~MASPInput();

	/// Getters
	// Number of agents
	int getN() {return N;}
	// Number of tasks
	int getM() {return M;}
	// Number of capabilities
	int getE() {return E;}
	// Capabilities per agent (input size: NxE)
	int get_c_ik(int i, int k) {return c_ik[i][k];}
	// Capability requirements per role (input size: MxE)
	int get_r_jk(int j, int k) {return r_jk[j][k];}
	// Number of agents required for task j (input size: M)
	int get_d_j(int j) {return d_j[j];}
	// Probability that agent i can complete task j (input size: NxN)
	double get_p_ij(int i, int j);
	// Get the x coordinate of task j (input size: M)
	double get_tpos_j_x(int j) {return tpos_j[j][0];}
	// Get the y coordinate of task j (input size: M)
	double get_tpos_j_y(int j) {return tpos_j[j][1];}
	// Get the x coordinate of drone i (input size: N)
	double get_apos_i_x(int i) {return apos_i[i][0];}
	// Get the y coordinate of drone i (input size: N)
	double get_apos_i_y(int i) {return apos_i[i][1];}
	// Determines if i is capable of performing j
	bool iCanDoj(int i, int j);
	// Determines a theoretical upper bound on a possible solution
	double UpperBound();
	std::string input_fileName;


private:
	// Number of agents
	int N;
	// Number of tasks
	int M;
	// Number of capabilities
	int E;
	// Capabilities per agent (size: NxE)
	int** c_ik;
	// Capability requirements per task (size: MxE)
	int** r_jk;
	// Probability that agent i can complete task j (size: NxM)
	double** p_ij;
	// Minimum number of agents to complete task j (size:M)
	int* d_j;
	// Position of task j (size:Mx2)
	double** tpos_j;
	// Position of drone i (size:Nx2)
	double** apos_i;

};
