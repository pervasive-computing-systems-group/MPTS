#include "MASP_MatchGS.h"


MASP_MatchGS::MASP_MatchGS() {
	if(SANITY_PRINT)
		printf("Hello from MASP_MatchAct Solver!\n");
	iterationCount = 0;
}


void MASP_MatchGS::Solve(MASPInput* input, I_solution* I_crnt) {
	// Initially, treat problem as bijection matching problem

	/*
	 * Format of cost map:
	 *       1   2   3   4   5   6 :agents
	 * 1.1 q11 q21 q31 q41 q51 q61
	 * 1.2 q11 q21 q31 q41 q51 q61
	 * 2.1 q12 q22 q32 q42 q52 q62
	 * 2.2 ...
	 * 2.3
	 * x     1   1   1   1   1   1
	 * :tasks
	 *
	 * The costs are the probability that an agent fails to
	 * complete a task q_ij = (1-pij)
	 */
	// Create cost matrix (NxN where cost[i][j] is the prob. that agent i fails at task j)
	std::vector<std::vector<double> > costMatrix;
	for(int i = 0; i < input->getN(); i ++) {
		std::vector<double> temp;
		for(int j = 0; j < input->getN(); j++) {
			int taskJ = get_task(j, input);
			if(taskJ >= 0) {
				if(input->iCanDoj(i, taskJ)) {
					// Regular task.. push probability of failure
					temp.push_back(1-input->get_p_ij(i, taskJ));
				}
				else {
					// I can't do j... assign a weight of inf
					temp.push_back(INF);
				}
			}
			else {
				// Floating task.. push probability of 1
				temp.push_back(1);
			}
		}
		costMatrix.push_back(temp);
	}

	// Sanity print...
	if(DEBUG_MASP_MCHGS) {
		printf("Cost Matrix:\n   ");
		for(int j = 0; j < input->getN(); j++) {
			printf("     %d", get_task(j, input));
		}
		printf("\n");
		for(long unsigned int i = 0; i < costMatrix.size(); i++) {
			printf(" %ld: ", i);
			for(long unsigned int j = 0; j < costMatrix.at(i).size(); j++) {
				printf(" %.3f", costMatrix.at(i).at(j));
			}
			printf("\n");
		}
	}

	// Run Hungarian algorithm
	vector<int> assignmentArray;
	mHungAlgo.Solve(costMatrix, assignmentArray);

	// Create local solution
	bool** x_ij = new bool*[input->getN()];
	for(int i = 0; i < input->getN(); i++) {
		x_ij[i] = new bool[input->getM()];
		for(int j = 0; j < input->getM(); j++) {
			x_ij[i][j] = false;
		}
	}
	double currentZ = 0;

	// Extract agent-task assignments
	for(int i = 0; i < input->getN(); i++) {
		int j = get_task(assignmentArray.at(i), input);
		if(j >= 0) {
			x_ij[i][j] = true;
		}
		else {
			// This agent was assigned to a dummy task...
		}
	}

	// Sanity print
	if(DEBUG_MASP_MCHGS) {
		printf("Solution after matching:\n");
		for(int i = 0; i < input->getN(); i++) {
			for(int j = 0; j < input->getM(); j++) {
				printf(" %d", x_ij[i][j]);
			}
		}
	}


	// While we are still making updates..
	iterationCount = 0;
	int iterationsWOChange = 0;
	while(iterationsWOChange < input->getN()) {
		bool madeChange = true;
		int index = iterationCount%input->getN();

		// Debug print
		if(DEBUG_MASP_MCHGS) {
			printf("Running agent %d\n", index);
		}

		// Record which task this agent was assigned to
		int currentJ = -1;
		for(int j = 0; j < input->getM(); j++) {
			if(x_ij[index][j]) {
				currentJ = j;
				x_ij[index][j] = false;
			}
		}

		// Debug print
		if(DEBUG_MASP_MCHGS) {
			printf(" previously assigned to %d\n", currentJ);
		}

		if(currentJ == -1) {
			// We weren't assigned yet, just join our favorite task for now...
			double bestP = -1;
			int bestJ = -1;
			for(int j = 0; j < input->getM(); j++) {
				if(input->iCanDoj(index, j)) {
					if(input->get_p_ij(index, j) > bestP) {
						bestP = input->get_p_ij(index, j);
						bestJ = j;
					}
				}
			}

			// Assign i to its favorite task
			x_ij[index][bestJ] = true;

			// Debug print
			if(DEBUG_MASP_MCHGS) {
				printf("  * just join favorite task %d\n", bestJ);
			}
		}
		else {
			double bestZ = -1;
			int bestJ = -1;
			// For each task...
			for(int j = 0; j < input->getM(); j++) {
				if(input->iCanDoj(index, j)) {
					// Try to assign this agent to the task
					x_ij[index][j] = true;
					// Check if this improved the performance
					double Z = BenchmarkCF(input, x_ij);
					if(Z > bestZ) {
						// Found a better spot
						bestZ = Z;
						bestJ = j;
					}
					// Reset
					x_ij[index][j] = false;
				}
			}
			// Assign the task that had the greatest impact
			x_ij[index][bestJ] = true;

			// Debug print
			if(DEBUG_MASP_MCHGS) {
				printf("  * Found best Z = %f at %d\n", bestZ, bestJ);
			}

			// Did we actually change anything...?
			if(bestJ == currentJ) {
				// We did not...
				madeChange = false;

				// Debug print
				if(DEBUG_MASP_MCHGS) {
					printf("   No change.. count = %d\n", iterationsWOChange);
				}
			}
		}

		iterationCount++;
		if(madeChange) {
			iterationsWOChange = 0;
		}
		else {
			iterationsWOChange++;
		}

		currentZ = BenchmarkCF(input, x_ij);
		if(SANITY_PRINT)
			printf("  Round %d, Z = %f\n", iterationCount, currentZ);
	}

	if(SANITY_PRINT) {
		printf("--------------------------------------------------------\n");
		printf("* Final Z = %f\n\n", currentZ);
	}

	// Save found solution
	for(int i = 0; i < input->getN(); i++) {
		for(int j = 0; j < input->getM(); j++) {
			if(x_ij[i][j]) {
				I_crnt->Update(input,i,j);
			}
		}
	}

	if(DEBUG_MASP_MCHGS)
		I_crnt->PrintSolution();

	// Free memory
	for(int i = 0; i < input->getN(); i++) {
		delete[] x_ij[i];
	}

	std::ofstream outfile("MatchGS.txt", std::ios::app);
	// outfile << "Input file: " << input->input_fileName << " has the following results: " << I_crnt->BenchmarkClsdForm() << std::endl;
	outfile << I_crnt->BenchmarkClsdForm() << std::endl;
	delete[] x_ij;
}


/*
 * Takes cost matrix index and converts it into the corresponding task's index. This assumes
 * that the entries in the matrix are in order based on minimum required agents per task
 * (e.g. 1.1 1.2 2.1 2.2 .. x.1 x.2 ..). It will return -1 if the given index is past the
 * minimum number of agents required over tasks (in x range from example).
 */
int MASP_MatchGS::get_task(int index, MASPInput* input) {
	// Verify that we were given a valid input
	if((index < 0) || (index >= input->getN())) {
		// Something went wrong -> hard fail!
		fprintf(stderr, "[MASP_MatchAct::get_task] : Bad index = %d\n", index);
		exit(1);
	}

	// Run through each task
	int run_j = 0;
	for(int j = 0; j < input->getM();j++) {
		for(int jj = 0; jj < input->get_d_j(j); jj++) {
			if(index == run_j) {
				return j;
			}
			run_j++;
		}
	}

	return -1;
}
