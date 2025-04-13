#include "MASP_EdgeCutting.h"


MASP_EdgeCutting::MASP_EdgeCutting() {
	if(SANITY_PRINT)
		printf("Hello from MASP Edge Cutting Solver!\n");
}


void MASP_EdgeCutting::Solve(MASPInput* input, I_solution* I_crnt) {
	// Create an initial solution using theoretical upper-bound logic
	bool** x_ij = new bool*[input->getN()];
	for(int i = 0; i < input->getN(); i++) {
		x_ij[i] = new bool[input->getM()];
		for(int j = 0; j < input->getM(); j++) {
			x_ij[i][j] = false;
		}
	}

	// Create an i to j mapping where any i that can do j is assigned to j
	for(int i = 0; i < input->getN(); i++) {
		// Check which tasks i can do
		for(int j = 0; j < input->getM(); j++) {
			// Can i do j?
			if(input->iCanDoj(i, j)) {
				x_ij[i][j] = true;
			}
			else {
				x_ij[i][j] = false;
			}
		}
	}

	// If any task has exactly d_j agents, free these agents from other assignments
	for(int j = 0; j < input->getM(); j++) {
		// Count have many agents are assigned to this task
		int agentsAssignedToTask = 0;
		std::vector<int> agentOnTask;
		for(int i = 0; i < input->getN(); i++) {
			if(x_ij[i][j]) {
				agentsAssignedToTask++;
				agentOnTask.push_back(i);
			}
		}
		// Is this the exact number required for the task?
		if(agentsAssignedToTask == input->get_d_j(j)) {
			// Sanity print
			if(DEBUG_MASP_EC)
				printf(" Found a set of agents to free\n");

			// Free these agents from other tasks
			for(int i : agentOnTask) {
				for(int jj = 0; jj < input->getM(); jj++) {
					if(j != jj) {
						x_ij[i][jj] = false;
					}
				}
			}
		}
	}

	// Sanity print
	if(DEBUG_MASP_EC) {
		printf("x_ij:\n");
		for(int i = 0; i < input->getN(); i++) {
			for(int j = 0; j < input->getM(); j++) {
				printf(" %d", x_ij[i][j]);
			}
			printf("\n");
		}
	}

	// For fun, let's determine the probability that each task is complete
	double upperBound = 1;
	for(int j = 0; j < input->getM(); j++) {
		double prob_j = 0;
		// Get the number of agents/probabilities of completion for this task
		std::vector<double> p_values;
		int agentsAssignedToTask = 0;
		for(int i = 0; i < input->getN(); i++) {
			if(x_ij[i][j]) {
				agentsAssignedToTask++;
				p_values.push_back(input->get_p_ij(i, j));
			}
		}
		for(int k = input->get_d_j(j); k <= agentsAssignedToTask; k++) {
			// Find probability that exactly k agents complete task j
			prob_j += m_PoissonB.PMF(k, agentsAssignedToTask, p_values);
		}
		upperBound *= prob_j;
	}

	// Sanity check...
	if(!floatEquality(upperBound, input->UpperBound())) {
		// Not what we expected... this should be the same logic
		fprintf(stderr," [ERROR] : MASP_EdgeCutting::Solve : Calculated upper-bound = %f != %f = input upper-bound\n", upperBound, input->UpperBound());
	}

	// While we still don't have a consistent solution...
	while(!validSolution(input, x_ij)) {
		// Sanity print
		if(DEBUG_MASP_EC)
			printf("Searching for the best task\n");

		// Pick best-performing task j' with more than d_j' agents and at least one agent can go to another task
		int bestJ = -1;
		double bestSuccess = 0;
		for(int j = 0; j < input->getM(); j++) {
			// Determine the probability that j is completed
			std::vector<double> p_values;
			int agentCount = 0;
			bool agentsCanBeKicked = false;
			for(int i = 0; i < input->getN(); i++) {
				if(x_ij[i][j]) {
					p_values.push_back(input->get_p_ij(i,j));
					agentCount++;

					// Check if i has somewhere to go if kicked...
					int taskCount = 0;
					for(int jj = 0; jj < input->getM(); jj++) {
						if(x_ij[i][jj]) {
							taskCount++;
						}
					}
					if(taskCount > 1) {
						agentsCanBeKicked = true;
					}
				}
			}

			// Does j have more agents than needed?
			if(agentsCanBeKicked && (agentCount > input->get_d_j(j))) {
				// Is j better off than all other qualifying tasks we have seen?
				double probJCompletes = m_PoissonB.P_s(input->get_d_j(j), agentCount, p_values);
				if(probJCompletes > bestSuccess) {
					// j is better, noted...
					bestJ = j;
					bestSuccess = probJCompletes;
				}
			}
		}

		if(bestJ == -1) {
			// We should have found an agent... hard fail
			fprintf(stderr, " [ERROR] : MASP_EdgeCutting::Solve : best task = -1\n");
			exit(1);
		}

		// Sanity print
		if(DEBUG_MASP_EC)
			printf(" found task %d, P_s = %f\n", bestJ, bestSuccess);

		if(DEBUG_MASP_EC)
			printf("Searching for least helpful agent\n");

		// Pick least-helpful agent i' assigned to j' that is also assigned to another task j''
		int agent_to_boot = -1;
		double agent_contribution = 1.01;
		for(int i = 0; i < input->getN(); i++) {
			if(x_ij[i][bestJ]) {
				// Sanity print
				if(DEBUG_MASP_EC)
					printf(" checking agent %d\n", i);

				// Count how many tasks this agent is assigned to
				int taskCount = 0;
				for(int j = 0; j < input->getM(); j++) {
					if(x_ij[i][j]) {
						taskCount++;
					}
				}
				// Can i be kicked out?
				if(taskCount > 1) {
					// Sanity print
					if(DEBUG_MASP_EC)
						printf("  %d is a candidate to boot...\n", i);

					// Determine how much i contributes to j
					std::vector<double> p_values;
					int agentCount = 0;
					for(int ii = 0; ii < input->getN(); ii++) {
						if(x_ij[ii][bestJ] && (i != ii)) {
							p_values.push_back(input->get_p_ij(i,bestJ));
							agentCount++;
						}
					}

					// Probability j completes without i
					double impactOfI = bestSuccess - m_PoissonB.P_s(input->get_d_j(bestJ), agentCount, p_values);
					// Sanity print
					if(DEBUG_MASP_EC)
						printf("  impact of %d : %f\n", i, impactOfI);
					if(impactOfI < agent_contribution) {
						// j is better, noted...
						agent_to_boot = i;
						agent_contribution = impactOfI;
					}
				}
			}
		}

		// Sanity print
		if(DEBUG_MASP_EC)
			printf(" found agent %d, d P_s = %f\n", agent_to_boot, agent_contribution);

		if(agent_to_boot == -1) {
			// We should have found an agent... hard fail
			fprintf(stderr, " [ERROR] : MASP_EdgeCutting::Solve : Agent to boot = -1\n");
			exit(1);
		}

		// Remove the selected agent from this task
		x_ij[agent_to_boot][bestJ] = false;

		// Sanity print
		if(DEBUG_MASP_EC) {
			printf("New x_ij:\n");
			for(int i = 0; i < input->getN(); i++) {
				for(int j = 0; j < input->getM(); j++) {
					printf(" %d", x_ij[i][j]);
				}
				printf("\n");
			}
		}
	}

	// Verify that we found a valid solution...

	// Assign agents to task in solution
	for(int i = 0; i < input->getN(); i++) {
		for(int j = 0; j < input->getM(); j++) {
			if(x_ij[i][j]) {
				// Assign i to task j
				I_crnt->Update(input, i, j);
			}
		}
	}

	// Memory cleanup
	for(int i = 0; i < input->getN(); i++) {
		delete[] x_ij[i];
	}
	delete[] x_ij;
}


// Returns true if each agent is assigned to at most one task
bool MASP_EdgeCutting::validSolution(MASPInput* input, bool** x_ij) {
	// Check each agent
	for(int i = 0; i < input->getN(); i++) {
		// Count the number of tasks i is assigned to
		int assignmentCount = 0;
		for(int j = 0; j < input->getM(); j++) {
			if(x_ij[i][j]) {
				assignmentCount++;
			}
		}

		// Is i assigned to more than one j?
		if(assignmentCount > 1) {
			// Assigned to more than one... this solution violates our constraints and is not valid!
			return false;
		}
	}

	// If we reach this point without violating the constraint.. then this is a valid solution
	return true;
}

