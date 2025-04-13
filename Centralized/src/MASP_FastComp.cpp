#include "MASP_FastComp.h"


MASP_FastComp::MASP_FastComp() {
	if(SANITY_PRINT)
		printf("Hello from MASPComp Solver!\n");
	fGlobalProbSuccess = 0;
	nInvalidCount = 0;
	nPruningCount = 0;
	bGlobalX_ij = NULL;
}


void MASP_FastComp::Solve(MASPInput* input, I_solution* I_crnt) {
	if(SANITY_PRINT)
		printf("\nStarting Complete solver\n");

	// Update current best-known solution
	fGlobalProbSuccess = 0;
	nInvalidCount = 0;
	nPruningCount = 0;

	// Create global best solution
	bGlobalX_ij = new bool*[input->getN()];
	// Create a local solution space
	bool** x_ij = new bool*[input->getN()];
	for(int i = 0; i < input->getN(); i++) {
		x_ij[i] = new bool[input->getM()];
		bGlobalX_ij[i] = new bool[input->getM()];
		for(int j = 0; j < input->getM(); j++) {
			x_ij[i][j] = false;
			bGlobalX_ij[i][j] = false;
		}
	}

	if(HEURISTIC_START) {
		// Find an initial solution using some heuristics-based method
		MASP_MatchGS heurstcSolver;
		I_solution I_heur(input);
		// Run heuristics solver
		heurstcSolver.Solve(input, &I_heur);
		// Extract the solution
		for(int i = 0; i < input->getN(); i++) {
			int j = I_heur.getTask(i);
			if(j >= 0) {
				bGlobalX_ij[i][j] = true;
			}
		}

		// Benchmark our starting solution
		fGlobalProbSuccess = BenchmarkCF(input, bGlobalX_ij);
		if(SANITY_PRINT) {
			printf(" Heuristic solution Z = %f\n", fGlobalProbSuccess);
		}
	}

	// Collect agents into a list - used for reordering later on
	if(DEBUG_MASP_FC) {
		printf(" Sorting Agents\n");
	}
	std::vector<FastAgent_t> sortedAgents;
	for(int i = 0; i < input->getN(); i++) {
		if(DEBUG_MASP_FC) {
			printf("  Adding agent: %d, ", i);
		}
		FastAgent_t agent(i,0);
		double average_p = 0;
		for(int j = 0; j < input->getM(); j++) {
			if(input->iCanDoj(i,j)) {
				agent.branchFactor++;
				average_p += input->get_p_ij(i,j);
			}
		}
		agent.average_p = average_p/(double)agent.branchFactor;
		if(DEBUG_MASP_FC) {
			printf("BF: %d, average P: %f\n", agent.branchFactor, agent.average_p);
		}
		sortedAgents.push_back(agent);
	}

	// Sanity print
	if(DEBUG_MASP_FC) {
		printf("Un-Sorted Agents:\n\tBranch Factor\tAgent-i\n");
		for(FastAgent_t agent : sortedAgents) {
			printf("\t%d\t-\t%d\n", agent.branchFactor, agent.agent_i);
		}
	}

	if(SORT_AGENTS) {
		// Record initial number of recursive calls
		long int oldRecStateSize = 0;
		long int newRecStateSize = 0;

		if(SANITY_PRINT) {
			// Record initial number of recursive calls
			oldRecStateSize = countRecursiveStates(sortedAgents);
		}

		std::sort(sortedAgents.begin(), sortedAgents.end());

		if(SANITY_PRINT) {
			// Record new number of recursive calls
			newRecStateSize = countRecursiveStates(sortedAgents);
			// Count the number of variable permutations (search space size)
			long int searchSpaceSize = 1;
			for(FastAgent_t agent : sortedAgents) {
				searchSpaceSize *= agent.branchFactor;
			}
			printf(" Recursive State Reduction: %ld -> %ld\n", oldRecStateSize, newRecStateSize);
			printf(" Size of search space: %ld\n", searchSpaceSize);
			printf(" Starting solver\n");
			printf("--------------------------------------------------------\n");
		}
	}

	// Sanity print
	if(DEBUG_MASP_FC) {
		printf("Sorted Agents:\n\tBranch Factor\tAgent-i\n");
		for(FastAgent_t agent : sortedAgents) {
			printf("\t%d\t-\t%d\n", agent.branchFactor, agent.agent_i);
		}
	}

	// Start recursive solver
	assign_next(input, 0, x_ij, I_crnt, sortedAgents);

	// Sanity print
	if(DEBUG_MASP_FC) {
		printf("\nBest found solution: Z(I, C) = %f\n", fGlobalProbSuccess);
		for(int i = 0; i < input->getN(); i++) {
			for(int j = 0; j < input->getM(); j++) {
				if(bGlobalX_ij[i][j]) {
					printf("%d->%d\n",i,j);
				}
			}
		}
	}

	// Save found solution
	for(int i = 0; i < input->getN(); i++) {
		for(int j = 0; j < input->getM(); j++) {
			if(bGlobalX_ij[i][j]) {
				I_crnt->Update(input,i,j);
			}
		}
	}

	if(DEBUG_MASP_FC)
		I_crnt->PrintSolution();

	// Free memory
	for(int i = 0; i < input->getN(); i++) {
		delete[] x_ij[i];
	}
	delete[] x_ij;
	for(int i = 0; i < input->getN(); i++) {
		delete[] bGlobalX_ij[i];
	}
	delete[] bGlobalX_ij;

	if(SANITY_PRINT) {
		printf(" * finished\n");
		printf("--------------------------------------------------------\n");
		printf("Final solution: %f\n", fGlobalProbSuccess);
		printf("Invalid solutions found: %ld\n", nInvalidCount);
		printf("Pruning count: %ld\n\n", nPruningCount);
	}
}

// i is the index into the agentMap - not the actual agents ID!
void MASP_FastComp::assign_next(MASPInput* input, int i, bool** x_ij, I_solution* I_crnt, std::vector<FastAgent_t>& agentMap) {
	// Check base case
	if(i >= input->getN()) {
		/// Hit base case
		// Verify this is a valid assignment
		if(valid_solution(input, x_ij)) {
			// Determine Z
			double Z = BenchmarkCF(input, x_ij);

			// Is this better than our best solution?
			if(Z > fGlobalProbSuccess) {
				// Sanity print
				if(SANITY_PRINT)
					printf(" * found improved solution: Z(I, C) = %f\n", Z);

				// Update best found solution
				fGlobalProbSuccess = Z;
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getM(); j++) {
						bGlobalX_ij[i][j] = x_ij[i][j];
					}
				}
			}
			else {
				// Sanity print
				if(DEBUG_MASP_FC)
					printf("Z(I, C) = %f\n", Z);
			}
		}
		else {
			// ELSE do nothing, not valid
			nInvalidCount++;
		}
	}
	else {
		/// Recursive case
		// Determine i for recursive step input
		int next_i = i + 1;
		// Iterate through all possible assignments of x_ij for input i
		for(int j = 0; j < input->getM(); j++) {
			// Verify that i meets the requirements of j
			if(input->iCanDoj(agentMap.at(i).agent_i, j)) {
				// Assign agent i to task j
				x_ij[agentMap.at(i).agent_i][j] = true;
				bool keepBranch = true;

				if(DEBUG_MASP_FC) {
					printf("-- Pruning check --\n");
				}

				// Pruning by requirements
				if(keepBranch) {
					// We only consider the set of unassigned agents (we just assigned i)
					int next_to_assign = i+1;
					int agents = input->getN() - next_to_assign;

					// Are there still any agents left un-assigned? (conversely, was i the last agent..?)
					if(agents > 0) {

						// Make a local copy of d_j
						std::vector<int> local_d_j;
						for(int local_j = 0; local_j < input->getM(); local_j++) {
							local_d_j.push_back(input->get_d_j(local_j));
						}

						// Determine how many tasks STILL require agents
						for(int local_i = 0; local_i < input->getN()-agents; local_i++) {
							// Determine which task this agent was assigned to
							for(int local_j = 0; local_j < input->getM(); local_j++) {
								int agent_i = agentMap.at(local_i).agent_i;
								if(x_ij[agent_i][local_j]) {
									local_d_j.at(local_j) = std::max(0, local_d_j.at(local_j) - 1);
								}
							}
						}

						// Determine a_r, the number of required agents
						int a_r = 0;
						for(int local_j = 0; local_j < input->getM(); local_j++) {
							a_r += local_d_j.at(local_j);
						}

						// Do we have enough agents...?
						if(a_r > agents) {
							// There aren't enough vacant agents to fill all requirements..
							keepBranch = false;

							// Sanity print
							if(DEBUG_MASP_FC) {
								printf("Not enough agents here:\nd_j:\n");
								for(int j = 0; j < input->getM(); j++) {
									printf("%d ", input->get_d_j(j));
								}
								printf("\nNumber not assigned: %d, number still needed: %d\n-- Prune by requirements --\n", agents, a_r);
							}
						}
					}
				}

				// Prune by bound
				if(keepBranch)  {
					// Pruning... search for a theoretical upper bound based on our current assignment
					double upperBound = 1;
					// Start by making a new solution space
					bool** xp_ij = new bool*[input->getN()];
					for(int i = 0; i < input->getN(); i++) {
						xp_ij[i] = new bool[input->getM()];
						for(int j = 0; j < input->getM(); j++) {
							xp_ij[i][j] = false;
						}
					}

					// Create an i to j mapping where any i' that can do j is assigned to j, for i' > input-i
					for(int ii = 0; ii < input->getN(); ii++) {
						int agent_i = agentMap.at(ii).agent_i;
						// Have we already assigned ii in input solution x_ij?
						if(ii <= i) {
							// Copy over the current assignment
							for(int j = 0; j < input->getM(); j++) {
								if(x_ij[agent_i][j]) {
									xp_ij[agent_i][j] = true;
								}
							}
						}
						else {
							// Check which tasks ii can do
							for(int j = 0; j < input->getM(); j++) {
								// Can i do j?
								if(input->iCanDoj(agent_i, j)) {
									xp_ij[agent_i][j] = true;
								}
								else {
									xp_ij[agent_i][j] = false;
								}
							}
						}
					}

					// Sanity print
					if(DEBUG_MASP_FC) {
						printf("nxp_ij:\n");
						for(int i = 0; i < input->getN(); i++) {
							for(int j = 0; j < input->getM(); j++) {
								printf(" %d", xp_ij[i][j]);
							}
							printf("\n");
						}
					}

					// Determine the probability that each task is complete
					for(int j = 0; j < input->getM(); j++) {
						double prob_j = 0;
						// Get the number of agents/probabilities of completion for this task
						std::vector<double> p_values;
						int agentsAssignedToTask = 0;
						for(int i = 0; i < input->getN(); i++) {
							if(xp_ij[i][j]) {
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

					// Sanity print
					if(DEBUG_MASP_FC) {
						printf("Upper-bound: %f, Incumbent solution: %f\n", upperBound, fGlobalProbSuccess);
						printf("-- Prune by bound? %d --\n", (upperBound < fGlobalProbSuccess));
					}

					// Can we prune this branch?
					if(upperBound < fGlobalProbSuccess) {
						// We won't find a better solution by branching here
						keepBranch = false;
						nPruningCount++;
					}

					// Memory cleanup
					for(int i = 0; i < input->getN(); i++) {
						delete[] xp_ij[i];
					}
					delete[] xp_ij;
				}

				// Should we recurse on this branch?
				if(keepBranch) {
					// Increment i and recurse
					assign_next(input, next_i, x_ij, I_crnt, agentMap);
				}

				// Reset i/j combo
				x_ij[agentMap.at(i).agent_i][j] = false;
			}
		}
	}
}

// Determine if this is a valid solution
bool MASP_FastComp::valid_solution(MASPInput* input, bool** x_ij) {
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

// Counts the number of recursive states required to iterate through (just FYI)
long int MASP_FastComp::countRecursiveStates(std::vector<FastAgent_t>& agents) {
	// The count starts at 1
	long int count = 1;
	long int previousSize = 1;
	for(long unsigned int i = 0; i < agents.size(); i++) {
		count += previousSize*agents.at(i).branchFactor;
		previousSize = previousSize*agents.at(i).branchFactor;
	}

	return count;
}


