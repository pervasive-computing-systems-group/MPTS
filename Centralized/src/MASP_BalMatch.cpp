#include "MASP_BalMatch.h"


MASP_BalMatch::MASP_BalMatch() {
	if(SANITY_PRINT)
		printf("Hello from MASP_BalMatch Solver!\n");
	srand (time(NULL));
	a_r=0;
	a_f=0;
	np=0;
}


void MASP_BalMatch::Solve(MASPInput* input, I_solution* I_crnt) {
	/*
	 * Treat problem as bijection matching problem
	 *
	 * costs
	 *    1.1 1.2 1.x 2.1 2.2 2.3 2.x :tasks
	 * 1  q11 q11 q11 q12 q12
	 * 2  ...
	 * 3
	 * 4
	 * 5
	 * 6
	 * 7'  1   1   0
	 * :agents
	 *
	 * q_ij = (1-p_ij)
	 */

	// Determine a_r, the number of required agents
	a_r = 0;
	for(int j = 0; j < input->getM(); j++) {
		a_r += input->get_d_j(j);
	}
	// Determine a_f, the number of floating agents
	a_f = input->getN() - a_r;
	// Determine np, the number of participants for balanced matching
	np = a_r + input->getM()*a_f;
	if(DEBUG_MASP_BM) {
		printf("n=%d, m=%d, a_r=%d, a_f=%d, np=%d\nTask Row:\n", input->getN(), input->getM(), a_r, a_f, np);
		for(int j = 0; j < input->getM();j++) {
			for(int i = 0; i < input->get_d_j(j); i++) {
				printf(" %d.%d", j, i);
			}
			for(int i = 0; i < a_f; i++) {
				printf(" %d.x", j);
			}
		}
		printf("\n");
	}

	// Create a cost map
	std::vector<std::vector<double> > costMatrix;
	for(int i = 0; i < np; i ++) {
		std::vector<double> temp;

		// Is this a real agent?
		int agentI = get_agent(i, input);
		if(agentI >= 0) {
			for(int j = 0; j < np; j++) {
				int taskJ = get_task(j, input);
				// Can i do j?
				if(input->iCanDoj(agentI, taskJ)) {
					temp.push_back(1-input->get_p_ij(agentI, taskJ));
				}
				else {
					// i can't do j... assign INF
					temp.push_back(INF);
				}
			}
		}
		else {
			// Phantom agent.. assign 1 for non-floating task and 0 for floating tasks
			for(int j = 0; j < np; j++) {
				if(floating_task(j, input)) {
					// Phantom agents prefer floating tasks
					temp.push_back(0);
				}
				else {
					// Phantoms do not prefer real tasks
					temp.push_back(INF);
				}
			}
		}
		costMatrix.push_back(temp);
	}

	// Sanity print
	if(DEBUG_MASP_BM) {
		printf("Cost map:\n");
		for(int j = 0; j < np; j ++) {
			for(int i = 0; i < np; i++) {
				printf(" %.2f", costMatrix.at(i).at(j));
			}
			puts("");
		}
	}

	vector<int> assignment;
	mHungAlgo.Solve(costMatrix,assignment);

	// Sanity print
	if(DEBUG_MASP_BM) {
		printf("Matchings:\n");
		for(long unsigned int i = 0; i < assignment.size(); i++) {
			printf(" %ld:%d\n", i, assignment.at(i));
		}
	}

	for(int i = 0; i < input->getN(); i++) {
		I_crnt->Update(input,i, get_task(assignment.at(i), input));
	}

	if(DEBUG_MASP_BM)
		I_crnt->PrintSolution();

	std::ofstream outfile("BalMatch.txt", std::ios::app);
	outfile << I_crnt->BenchmarkClsdForm() << std::endl;
}

// Takes balanced matching index and converts it into the corresponding task's index
int MASP_BalMatch::get_task(int bal_j, MASPInput* input) {
	int run_j = 0;
	for(int j = 0; j < input->getM();j++) {
		for(int i = 0; i < input->get_d_j(j); i++) {
			if(bal_j == run_j) {
				return j;
			}
			run_j++;
		}
		for(int i = 0; i < a_f; i++) {
			if(bal_j == run_j) {
				return j;
			}
			run_j++;
		}
	}

	// If you made it here... something went wrong -> hard fail!
	fprintf(stderr, "[MASP_BalMatch::get_task] : Could not find mapping from bal_j to task_j, bal_j = %d\n", bal_j);
	exit(1);
}

// Takes balanced matching index and converts it into the corresponding agent's index.
// Returns -1 if this is a phantom
int MASP_BalMatch::get_agent(int i, MASPInput* input) {
	if(i < input->getN()) {
		return i;
	}

	// This is a phantom agent
	return -1;
}

// Determines if this is a floating task
bool MASP_BalMatch::floating_task(int bal_j, MASPInput* input) {
	int run_j = 0;
	for(int j = 0; j < input->getM();j++) {
		for(int i = 0; i < input->get_d_j(j); i++) {
			if(bal_j == run_j) {
				return false;
			}
			run_j++;
		}
		for(int i = 0; i < a_f; i++) {
			if(bal_j == run_j) {
				return true;
			}
			run_j++;
		}
	}

	// If you made it here... something went wrong -> hard fail!
	fprintf(stderr, "[MASP_BalMatch::floating_task] : Could not find mapping from bal_j to task_j, bal_j = %d\n", bal_j);
	exit(1);
}

int MASP_BalMatch::resolve_task(int jj, MASPInput* input) {
	int j = 0;
	int j_track = 0;

	while(j < input->getM()) {
		if(jj >= j_track && jj < (j_track+input->get_d_j(j))) {
			return j;
		}

		// Update j and j-tracker
		j_track += input->get_d_j(j);
		j++;
	}

	// This must be a floating task
	return -1;
}
