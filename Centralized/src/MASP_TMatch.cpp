#include "MASP_TMatch.h"


MASP_TMatch::MASP_TMatch() {
	if(SANITY_PRINT)
		printf("Hello from MASP_TMatch Solver!\n");
	srand (time(NULL));
}


void MASP_TMatch::Solve(MASPInput* input, I_solution* I_crnt) {
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
	if(DEBUG_MASP_TMCH) {
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
	if(DEBUG_MASP_TMCH) {
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
	if(DEBUG_MASP_TMCH) {
		printf("P_j: [");
		for(int j = 0; j < input->getM(); j++) {
			printf(" %f", P_j[j]);
		}
		printf("]\n");
	}

	/*
	 * Start weighted greedy part. The task with the lowest probability of
	 * failure gets first pick of remaining agents. If the lowest cannot
	 * match with any floating agents, then we give the option to the next
	 * lowest... and so on.
	 */

	// Sanity print
	if(DEBUG_MASP_TMCH)
		printf("Starting weighted greedy pick\n");

	// Create a blacklist. Tasks on the blacklist cannot pick agents from
	// I_e because no agent in I_e can help them...
	bool* back_list = new bool[input->getM()];
	for(int j = 0; j < input->getM(); j++) {
		back_list[j] = false;
	}

	// Run until there are no more floating agents to assign to tasks
	while(!I_e.empty()) {
		bool made_progress = false;
		while(!made_progress) {
			// Determine which task has the lowest probability of completion
			int jj = -1;
			double worst_success = 1.1;
			for(int j = 0; j < input->getM(); j++) {
				// Check probability of success for task j
				if(!back_list[j] && P_j[j] < worst_success) {
					worst_success = P_j[j];
					jj = j;
				}
			}

			if(jj == -1) {
				// Something went wrong here...
				printf("ERROR [MASP_TMatch::Solve] : All tasks blacklisted... Still need to assign %d\n", I_e.front());
				exit(0);
			}

			// Sanity print
			if(DEBUG_MASP_TMCH)
				printf(" Letting %d pick an agent\n", jj);

			// Let jj pick an agent
			int agent_pick = -1;
			double best_success = 0;
			for(int i : I_e) {
				if((input->get_p_ij(i,jj) > best_success) && (input->iCanDoj(i, jj))) {
					best_success = input->get_p_ij(i,jj);
					agent_pick = i;
				}
			}

			// Did jj find an agent?
			if(agent_pick >= 0) {
				// Assign the agent to jj
				combos.at(jj).push_back(agent_pick);
				I_e.remove(agent_pick);
				made_progress = true;

				// Sanity print
				if(DEBUG_MASP_TMCH)
					printf(" %d picked %d\nRunning Monte-Carlo Sim, initially P_%d = %f\n", jj, agent_pick, jj, P_j[jj]);

				// Re-calculate P_j for jj
				P_j[jj] = monteSim(input, combos.at(jj), jj, I_crnt);

				// Sanity print
				if(DEBUG_MASP_TMCH)
					printf(" Post Monte-Carlo Sim, P_%d = %f\n", jj, P_j[jj]);
			}
			else {
				// No available agents can help jj... blacklist this task
				back_list[jj] = true;

				// Sanity print
				if(DEBUG_MASP_TMCH)
					printf(" %d failed, blacklist them..\n", jj);
			}
		}
	}

	// Extract solution from I_a and store it in x_ij
	for(int j = 0; j < input->getM(); j++) {
		for(auto i : combos.at(j)) {
			I_crnt->Update(input,i,j);
		}
	}

	if(DEBUG_MASP_TMCH)
		I_crnt->PrintSolution();

	// Memory cleanup
	delete[] P_j;
	delete[] back_list;
}

// Runs Monti Carlo Simulation to determine the probability that the agents in I_j complete task j
double MASP_TMatch::monteSim(MASPInput* input, std::vector<int>& I_j, int j, I_solution* I_curnt) {
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

/*
 * Takes cost matrix index and converts it into the corresponding task's index. This assumes
 * that the entries in the matrix are in order based on minimum required agents per task
 * (e.g. 1.1 1.2 2.1 2.2 .. x.1 x.2 ..). It will return -1 if the given index is past the
 * minimum number of agents required over tasks (in x range from example).
 */
int MASP_TMatch::get_task(int index, MASPInput* input) {
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
