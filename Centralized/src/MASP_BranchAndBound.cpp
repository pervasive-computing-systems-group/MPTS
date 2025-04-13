#include "MASP_BranchAndBound.h"


MASP_BranchAndBound::MASP_BranchAndBound() {
	if(SANITY_PRINT)
		printf("Hello from MASPComp Solver!\n");
	fGlobalProbSuccess = 0;
	nInvalidCount = 0;
	nPruningCount = 0;
	bGlobalX_ij = NULL;
}


void MASP_BranchAndBound::Solve(MASPInput* input, I_solution* I_crnt) {
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

	// Collect agents into a list - used for reordering later on
	if(DEBUG_MASP_BNB) {
		printf(" Sorting Agents\n");
	}
	std::vector<BnBAgent_t> sortedAgents;
	for(int i = 0; i < input->getN(); i++) {
		if(DEBUG_MASP_BNB) {
			printf("  Adding agent: %d, ", i);
		}
		BnBAgent_t agent(i,0);
		double average_p = 0;
		for(int j = 0; j < input->getM(); j++) {
			if(input->iCanDoj(i,j)) {
				agent.branchFactor++;
				average_p += input->get_p_ij(i,j);
			}
		}
		agent.average_p = average_p/(double)agent.branchFactor;
		if(DEBUG_MASP_BNB) {
			printf("BF: %d, average P: %f\n", agent.branchFactor, agent.average_p);
		}
		sortedAgents.push_back(agent);
	}

	// Sanity print
	if(DEBUG_MASP_BNB) {
		printf("Un-Sorted Agents:\n\tBranch Factor\tAgent-i\n");
		for(BnBAgent_t agent : sortedAgents) {
			printf("\t%d\t-\t%d\n", agent.branchFactor, agent.agent_i);
		}
	}

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
		for(BnBAgent_t agent : sortedAgents) {
			searchSpaceSize *= agent.branchFactor;
		}
		printf(" Recursive State Reduction: %ld -> %ld\n", oldRecStateSize, newRecStateSize);
		printf(" Size of search space: %ld\n", searchSpaceSize);
		printf(" Starting solver\n");
		printf("--------------------------------------------------------\n");
	}


	// Sanity print
	if(DEBUG_MASP_BNB) {
		printf("Sorted Agents:\n\tBranch Factor\tAgent-i\n");
		for(BnBAgent_t agent : sortedAgents) {
			printf("\t%d\t-\t%d\n", agent.branchFactor, agent.agent_i);
		}
	}

	// Start recursive solver
	assign_next(input, 0, x_ij, I_crnt, sortedAgents);

	// Sanity print
	if(DEBUG_MASP_BNB) {
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

	if(DEBUG_MASP_BNB)
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
void MASP_BranchAndBound::assign_next(MASPInput* input, int i, bool** x_ij, I_solution* I_crnt, std::vector<BnBAgent_t>& agentMap) {
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
				if(DEBUG_MASP_BNB)
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

				// Pruning by solution existence
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
							if(DEBUG_MASP_BNB) {
								printf("Not enough agents here:\nd_j:\n");
								for(int j = 0; j < input->getM(); j++) {
									printf("%d ", input->get_d_j(j));
								}
								printf("\nNumber not assigned: %d, number still needed: %d\n-- Prune by requirements --\n", agents, a_r);
							}
						}
					}
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
bool MASP_BranchAndBound::valid_solution(MASPInput* input, bool** x_ij) {
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
long int MASP_BranchAndBound::countRecursiveStates(std::vector<BnBAgent_t>& agents) {
	// The count starts at 1
	long int count = 1;
	long int previousSize = 1;
	for(long unsigned int i = 0; i < agents.size(); i++) {
		count += previousSize*agents.at(i).branchFactor;
		previousSize = previousSize*agents.at(i).branchFactor;
	}

	return count;
}



// Takes balanced matching index and converts it into the corresponding agent's index.
// Returns -1 if this is a phantom
int MASP_BranchAndBound::get_agent(int i, int N) {
	if(i < N) {
		return i;
	}

	// This is a phantom agent
	return -1;
}

// Takes balanced matching index and converts it into the corresponding task's index
int MASP_BranchAndBound::get_task(int bal_j, int M, int a_f, std::vector<int>& d_j) {
	int run_j = 0;
	for(int j = 0; j < M; j++) {
		for(int i = 0; i < d_j.at(j); i++) {
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

// Determines if this is a floating task
bool MASP_BranchAndBound::floating_task(int bal_j, int M, int a_f, std::vector<int>& d_j) {
	int run_j = 0;
	for(int j = 0; j < M; j++) {
		for(int i = 0; i < d_j.at(j); i++) {
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


