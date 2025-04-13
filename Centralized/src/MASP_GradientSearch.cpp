#include "MASP_GradientSearch.h"


MASP_GradientSearch::MASP_GradientSearch() {
	if(SANITY_PRINT)
		printf("Hello from MASPComp Solver!\n");
	iterationCount = 0;
}


void MASP_GradientSearch::Solve(MASPInput* input, I_solution* I_crnt) {
	if(SANITY_PRINT)
		printf("\nStarting Gradient Search solver\n--------------------------------------------------------\n");

	// Create local solution
	bool** x_ij = new bool*[input->getN()];
	for(int i = 0; i < input->getN(); i++) {
		x_ij[i] = new bool[input->getM()];
		for(int j = 0; j < input->getM(); j++) {
			x_ij[i][j] = false;
		}
	}
	double currentZ = 0;

	// While we are still making updates..
	iterationCount = 0;
	int iterationsWOChange = 0;
	while(iterationsWOChange < input->getN()) {
		bool madeChange = true;
		int index = iterationCount%input->getN();

		// Debug print
		if(DEBUG_MASP_GS) {
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
		if(DEBUG_MASP_GS) {
			printf(" previously assigned to %d\n", currentJ);
		}

		// Attempt to join any suitable task that has < d_j agents
		bool joinedTask = false;
		{
			// Check our favorite task, if it already has d_j agents then search next fav task o.w assign to fav
			std::priority_queue<std::tuple<double, int>> pQ;
			for(int j = 0; j < input->getM(); j++) {
				if(input->iCanDoj(index, j)) {
					std::tuple<double, int> tup(input->get_p_ij(index, j), j);
					pQ.push(tup);
				}
			}

			// Try to join a task
			while(!pQ.empty() && !joinedTask) {
				// Get our next favorite task
				auto n = pQ.top();
				int j = std::get<1>(n);
				pQ.pop();

				// Sanity print
				if(DEBUG_MASP_GS)
					printf(" Agent %d wants task %d, with %f\n", index, std::get<1>(n), std::get<0>(n));

				// How man agents are already assigned to j?
				int assigedToJ = 0;
				for(int i = 0; i < input->getN(); i++) {
					if(x_ij[i][j]) {
						assigedToJ++;
					}
				}

				// If there is still room for this agent...
				if(assigedToJ < input->get_d_j(j)) {
					// Assign this agent to j
					x_ij[index][j] = true;
					joinedTask = true;

					// Debug print
					if(DEBUG_MASP_GS) {
						printf(" * joining task %d\n", j);
					}

					// Did we actually change anything...?
					if(j == currentJ) {
						// We did not...
						madeChange = false;

						// Debug print
						if(DEBUG_MASP_GS) {
							printf("   No change.. count = %d\n", iterationsWOChange);
						}
					}
				}
				else {
					// Try the next task...
				}
			}
		}

		// If no such task exists... Cycle through each to see if we can make an improvement
		if(!joinedTask) {
			// Debug print
			if(DEBUG_MASP_GS) {
				printf(" All easy entries are full...\n");
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
				if(DEBUG_MASP_GS) {
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
				if(DEBUG_MASP_GS) {
					printf("  * Found best Z = %f at %d\n", bestZ, bestJ);
				}

				// Did we actually change anything...?
				if(bestJ == currentJ) {
					// We did not...
					madeChange = false;

					// Debug print
					if(DEBUG_MASP_GS) {
						printf("   No change.. count = %d\n", iterationsWOChange);
					}
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

	if(DEBUG_MASP_GS)
		I_crnt->PrintSolution();

	// Free memory
	for(int i = 0; i < input->getN(); i++) {
		delete[] x_ij[i];
	}

	std::ofstream outfile("GradientSearch.txt", std::ios::app);
	outfile << I_crnt->BenchmarkClsdForm() << std::endl;
	delete[] x_ij;
}

