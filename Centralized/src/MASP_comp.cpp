#include "MASP_comp.h"


MASPComp::MASPComp() {
	if(SANITY_PRINT)
		printf("Hello from MASPComp Solver!\n");
	fProbSuccess = 0;
	bX_ij = NULL;
}


void MASPComp::Solve(MASPInput* input, I_solution* I_crnt) {
	// Create global best solution
	bX_ij = new bool*[input->getN()];
	// Create a local solution space
	bool** x_ij = new bool*[input->getN()];
	for(int i = 0; i < input->getN(); i++) {
		x_ij[i] = new bool[input->getM()];
		bX_ij[i] = new bool[input->getM()];
		for(int j = 0; j < input->getM(); j++) {
			x_ij[i][j] = false;
			bX_ij[i][j] = false;
		}
	}
	fProbSuccess = 0;

	// Start recursive solver
	assign_next(input, 0, x_ij, I_crnt);

	// Sanity print
	if(DEBUG_MASPC) {
		printf("\nBest found solution: Z(I, C) = %f\n", fProbSuccess);
		for(int i = 0; i < input->getN(); i++) {
			for(int j = 0; j < input->getM(); j++) {
				if(bX_ij[i][j]) {
					printf("%d->%d\n",i,j);
				}
			}
		}
	}

	// Save found solution
	for(int i = 0; i < input->getN(); i++) {
		for(int j = 0; j < input->getM(); j++) {
			if(bX_ij[i][j]) {
				I_crnt->Update(input,i,j);
			}
		}
	}

	if(DEBUG_MASPC)
		I_crnt->PrintSolution();

	// Free memory
	for(int i = 0; i < input->getN(); i++) {
		delete[] x_ij[i];
	}
	delete[] x_ij;
	for(int i = 0; i < input->getN(); i++) {
		delete[] bX_ij[i];
	}
	delete[] bX_ij;
}

void MASPComp::assign_next(MASPInput* input, int i, bool** x_ij, I_solution* I_crnt) {
	// Check base case
	if(i >= input->getN()) {
		/// Hit base case
		// Verify this is a valid assignment
		if(valid_solution(input, x_ij)) {
			// Determine Z
			double Z = BenchmarkRec(input, x_ij);

			// Is this better than our best solution?
			if(Z > fProbSuccess) {
				// Sanity print
				if(DEBUG_MASPC)
					printf("** Found better solution! Z(I, C) = %f\n", Z);

				// Update best found solution
				fProbSuccess = Z;
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getM(); j++) {
						bX_ij[i][j] = x_ij[i][j];
					}
				}
			}
			else {
				// Sanity print
				if(DEBUG_MASPC)
					printf("Z(I, C) = %f\n", Z);
			}
		}
		// ELSE do nothing, not valid
	}
	else {
		/// Recursive case
		// Determine i for recursive step input
		int next_i = i+1;
		// Iterate through all possible assignments of x_ij for input i
		for(int j = 0; j < input->getM(); j++) {
			// Verify that i meets the requirements of j
			if(input->iCanDoj(i,j)) {
				// TODO: Verify that there are enough agents to cover other task
				// Assign agent i to task j
				x_ij[i][j] = true;
				// Increment i and recurse
				assign_next(input, next_i, x_ij, I_crnt);
			}
			// Reset i/j combo
			x_ij[i][j] = false;
		}
	}
}

// Determine if this is a valid solution
bool MASPComp::valid_solution(MASPInput* input, bool** x_ij) {
	// Count the number of agents assigned to each task
	for(int j = 0; j < input->getM(); j++) {
		int agent_count = 0;
		// Cycle through all agents
		for(int i = 0; i < input->getN(); i++) {
			if(x_ij[i][j]) {
				agent_count++;
			}
		}

		// Check if we assigned enough agents to this task
		if(agent_count < input->get_d_j(j)) {
			// Not enough agents assigned to this task
			return false;
		}
	}

	// If we make it this far, all tasks must have enough agents
	return true;
}



