#include "Solver.h"

using namespace std::complex_literals;

Solver::Solver() {
}

Solver::~Solver() {}



/*
 * Determines the probability of mission success given the input problem
 * and assignments in x_ij. Assumes that x_ij contains a valid solution.
 */
double Solver::BenchmarkRec(MASPInput* input, bool** x_ij) {
	// Sanity print
	if(DEBUG_SOLVER) {
		printf("Found Solution I: ");
		for(int i = 0; i < input->getN(); i++) {
			for(int j = 0; j < input->getM(); j++) {
				if(x_ij[i][j]) {
					printf("%d->%d ", i, j);
				}
			}
		}
		puts("");
	}

	// Probability that all tasks are complete
	double prob_success = 1;

	// Count the number of agents assigned to each task
	for(int j = 0; j < input->getM(); j++) {
		std::vector<int> I_j;
		int agent_count = 0;
		// Cycle through all agents
		for(int i = 0; i < input->getN(); i++) {
			if(x_ij[i][j]) {
				agent_count++;
				I_j.push_back(i);
			}
		}

		// Calculate the probability that d_j or more agents complete the task
		prob_success *= P_j(input, I_j, j);
	}

	// Sanity print
	if(DEBUG_SOLVER)
		printf("* p_a(I) = %f\n", prob_success);

	return prob_success;
}

/*
 * Determines the probability of mission success given the input problem
 * and assignment I_sol. Assumes that I_sol contains a valid solution.
 */
double Solver::BenchmarkCF(MASPInput* input, bool** x_ij) {
	// Sanity print
	if(DEBUG_SOLVER) {
		printf("Found Solution I: ");
		for(int i = 0; i < input->getN(); i++) {
			for(int j = 0; j < input->getM(); j++) {
				if(x_ij[i][j]) {
					printf("%d->%d ", i, j);
				}
			}
		}
		puts("");
	}

	// Probability that all tasks are complete
	double prob_success = 1;

	// For each task
	for(int j = 0; j < input->getM(); j++) {
		if(DEBUG_SOLVER)
			printf(" + {1");

		// Determine the number of agents assigned to j (N_j)
		int N_j = 0;
		for(int i = 0; i < input->getN(); i++) {
			if(x_ij[i][j]) {
				N_j ++;
			}
		}
		// Assemble a list of probabilities that each agent is successful
		std::vector<double> p_values;
		for(int i = 0; i < input->getN(); i++) {
			if(x_ij[i][j]) {
				p_values.push_back(input->get_p_ij(i,j));
			}
		}

		// Calculate the Poisson-binomial success function for k from d_j up to the number of agents assigned to j
		double Ps = 0;
		// For k in {d_j ... n_j}
		for(int k = input->get_d_j(j); k <= N_j; k++) {
			// Determine the probability that exactly k agents perform task j
			if(DEBUG_SOLVER)
				printf(" * P_f(K=%d, I_%d)", k, j);

			// PMF - Probability of getting k successes out of n trials
			Ps += m_poissonBinomial.PMF(k, N_j, p_values);
		}
		if(DEBUG_SOLVER)
			printf("}");

		// Probability of complete success is the product of Ps(k, I_j) for k from d_j to n_j
		prob_success *= Ps;

		// If we hit zero... just give up
		if(isZero(prob_success)) {
			break;
		}
	}
	if(DEBUG_SOLVER)
		printf(" = %f\n", prob_success);

	return prob_success;
}

bool Solver::wholeNumber(double f) {
	double diff = f-floor(f);
	if(diff >= 0) {
		return diff < 0.0005;
	}
	else {
		return diff > -0.0005;
	}
}

// Calculates P_j for the agents in I_j
double Solver::P_j(MASPInput* input, std::vector<int>& I_j, int j) {
	// Basic function parameteres
	int agent_count = static_cast<int>(I_j.size());
	double fP_j = 0;

	// Start with the probability that S_d_j agents can do the task, then increment
	for(int l = input->get_d_j(j), i = 0; l <= agent_count; l++, i++) {
		// Calculate S_l
		fP_j += pow(-1, i)*nChoosek(l-1, i)*S_l(input, I_j, j, l);
	}

	// Sanity print
	if(DEBUG_SOLVER)
		printf("P_%d(I_j) = ", j);
	for(int l = input->get_d_j(j), i = 0; l <= agent_count; l++, i++) {
		// Sanity print
		if(DEBUG_SOLVER)
			printf("(%f)%dS_%d + ", pow(-1, i), nChoosek(l-1, i), l);
	}
	if(DEBUG_SOLVER)
		printf("0 = %f\n", fP_j);

	return fP_j;
}

int Solver::nChoosek(int n, int k) {
	if (k > n) return 0;
	if (k * 2 > n) k = n-k;
	if (k == 0) return 1;

	int result = n;
	for( int i = 2; i <= k; ++i ) {
		result *= (n-i+1);
		result /= i;
	}

	return result;
}

// Calculate S_l
double Solver::S_l(MASPInput* input, std::vector<int>& I_j, int j, int l) {
	double fS_l = 0;

	// Sanity print
	if(DEBUG_SOLVER) {
		printf("Finding S_%d for list: {", l);
		for(int n : I_j) {
			printf(" %d", n);
		}
		printf("}\n");
	}

	// Basic function setup
	std::vector<std::vector<int>> combos;
	int* arr = new int[I_j.size()];
	bool* check = new bool[I_j.size()];

	// Take the input of the array.
	for(int i = 0; i < static_cast<int>(I_j.size()); i++) {
		arr[i] = I_j.at(i);
		check[i] = false;
	}

	// Compile a list of lists, each with size l and containing a
	// unique combination of the elements in I_j
	combination(arr, l, 0, 0, check, static_cast<int>(I_j.size()), combos);

	// Sanity print
	if(DEBUG_SOLVER) {
		printf("Elements in combos:\n");
		for(std::vector<int> list : combos) {
			printf("{");
			for(int n : list) {
				printf(" %d", n);
			}
			printf("}");
		}
		puts("");
	}

	// Calculate S_l
	for(std::vector<int> list : combos) {
		double list_prob = 1;
		for(int i : list) {
			list_prob *= input->get_p_ij(i, j);
			// Sanity print
			if(DEBUG_SOLVER)
				printf("(%f)", input->get_p_ij(i, j));
		}
		fS_l += list_prob;
		// Sanity print
		if(DEBUG_SOLVER)
			printf(" + ");
	}

	// Sanity print
	if(DEBUG_SOLVER)
		printf("0\nS_%d(I_%d) = %f\n", l, j, fS_l);

	return fS_l;
}

// Compiles all combinations of length reqLen in list a
void Solver::combination(int* a, int reqLen, int start, int currLen, bool* check, int len, std::vector<std::vector<int>>& combos) {
	// Return if the currLen is more than the required length
	if(currLen > reqLen) {
		return;
	}
	else if (currLen == reqLen) {
		// currLen is equal to required length -> record sequence
		std::vector<int> tempList;
		for (int i = 0; i < len; i++) {
			if (check[i] == true) {
				tempList.push_back(a[i]);
			}
		}

		// Store this list of combinations
		combos.push_back(tempList);
		return;
	}

	// If start equals to len then return since no further element left.
	if (start == len) {
		return;
	}

	// For every index we have two options.
	// First is, we select it, means put true in check[] and increment currLen and start.
	check[start] = true;
	combination(a, reqLen, start + 1, currLen + 1, check, len, combos);
	// Second is, we don't select it, means put false in check[] and only start incremented.
	check[start] = false;
	combination(a, reqLen, start + 1, currLen, check, len, combos);
}
