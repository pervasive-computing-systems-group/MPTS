#include "MASP_MatchAct.h"


MASP_MatchAct::MASP_MatchAct() {
	if(SANITY_PRINT)
		printf("Hello from MASP_MatchAct Solver!\n");
	srand (time(NULL));
}


void MASP_MatchAct::Solve(MASPInput* input, I_solution* I_crnt) {
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
			if(get_task(j, input) >= 0) {
				// Regular task.. push probability of failure
				temp.push_back(1-input->get_p_ij(i, get_task(j, input)));
			}
			else {
				// Floating task.. push probability of 1
				// TODO: Should this be 0?
				temp.push_back(1);
			}
		}
		costMatrix.push_back(temp);
	}

	// Sanity print...
	if(DEBUG_MASP_MCHAC) {
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

	std::vector<std::vector<int>> combos;
	for(int j = 0; j < input->getM(); j++) {
		std::vector<int> temp;
		combos.push_back(temp);
	}
	std::list<int> I_e;

	// Extract agent-task assignments
	for(int i = 0; i < input->getN(); i++) {
		if(get_task(assignmentArray.at(i), input) >= 0) {
			combos.at(get_task(assignmentArray.at(i), input)).push_back(i);
		}
		else {
			// This agent was assigned to a dummy task...
			I_e.push_back(i);
		}
	}

	// Sanity print
	if(DEBUG_MASP_MCHAC) {
		printf("I_a = {");
		for(std::vector<int> list : combos) {
			printf("{");
			for(int n : list) {
				printf(" %d", n);
			}
			printf("}");
		}
		printf("}\nI_e = {");
		for(int n : I_e) {
			printf(" %d", n);
		}
		printf("}\n");
	}

	// Determine the probability that each task is complete. Because each task
	// has exactly d_j agents, P_j = p_1*p_2* ... p_n, for p_i in I_j
	double* P_j = new double[input->getM()];
	for(int j = 0; j < input->getM(); j++) {
		double p_j = 1;
		for(int i : combos.at(j)) {
			p_j *= input->get_p_ij(i,j);
		}
		P_j[j] = p_j;
	}

	// Sanity print
	if(DEBUG_MASP_MCHAC) {
		printf("P_j: [");
		for(int j = 0; j < input->getM(); j++) {
			printf(" %f", P_j[j]);
		}
		printf("]\n");
	}

	// Run blind auction
	while(!I_e.empty()) {
		int i = I_e.back();
		I_e.pop_back();
		// How much is each task willing to pay for this task?
		std::vector<double> bid;
		for(int j = 0; j < input->getM(); j++) {
			// Add i to j
			combos.at(j).push_back(i);
			// The bid is estimated <I_j with i> - <current I_j>
			double est_Pi = monteSim(input, combos.at(j), j, I_crnt);
			// Take i back away
			combos.at(j).pop_back();
			// Let j make a bid
			double bid_i = est_Pi-P_j[j];
			bid.push_back(bid_i);
		}

		// Who gets i?
		int winning_j = -1;
		double highest_bid = -1;
		for(auto j = 0; j < input->getM(); j++) {
			if(bid.at(j) > highest_bid) {
				highest_bid = bid.at(j);
				winning_j = j;
			}
		}

		// Add i to winning_j
		combos.at(winning_j).push_back(i);
		// Re-calculate P_j for winning_j
		P_j[winning_j] = monteSim(input, combos.at(winning_j), winning_j, I_crnt);

		if(DEBUG_MASP_MCHAC)
			printf(" %d has winning bid at %f\n", winning_j, highest_bid);
	}

	// Extract solution from I_a and store it in x_ij
	for(int j = 0; j < input->getM(); j++) {
		for(auto i : combos.at(j)) {
			I_crnt->Update(input,i,j);
		}
	}

	if(DEBUG_MASP_MCHAC)
		I_crnt->PrintSolution();

	// Memory cleanup
	std::ofstream outfile("MatchAct.txt", std::ios::app);
	outfile << I_crnt->BenchmarkClsdForm() << std::endl;
	delete[] P_j;

}


/*
 * Takes cost matrix index and converts it into the corresponding task's index. This assumes
 * that the entries in the matrix are in order based on minimum required agents per task
 * (e.g. 1.1 1.2 2.1 2.2 .. x.1 x.2 ..). It will return -1 if the given index is past the
 * minimum number of agents required over tasks (in x range from example).
 */
int MASP_MatchAct::get_task(int index, MASPInput* input) {
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

// Runs Monti Carlo Simulation to determine the probability that the agents in I_j complete task j
double MASP_MatchAct::monteSim(MASPInput* input, std::vector<int>& I_j, int j, I_solution* I_curnt) {
	double chance_of_success = 0;

	// Setup simulation
	int N = 500;
	int total_successes = 0;

	// Run N simulations
	for(int l = 0; l < N; l++) {
		int run_successes = 0;
		// Cycle through each agent
		for(int i : I_j) {
			// Take a random outcome
			double rand_outcome = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			if(input->get_p_ij(i,j) > rand_outcome) {
				// Random outcome was within p_ij -> Agent succeeded!
				run_successes++;
			}
		}

		// Did enough agents succeed?
		if(run_successes >= input->get_d_j(j)) {
			// Enough agents succeeded, task was complete!
			total_successes++;
		}
	}
	chance_of_success = total_successes/double(N);

	return chance_of_success;
}
