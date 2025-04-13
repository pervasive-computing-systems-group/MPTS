/*
 * I_solution.cpp
 *
 *  Created on: Oct 7, 2023
 *      Author: jonathan
 */

#include "I_solution.h"

I_solution::I_solution(MASPInput* input) {
	// Initialize the size information
	m_input = input;
	m_N = m_input->getN();
	m_M = m_input->getM();

	if(DEBUG_I_SOL)
		printf("New Solution I from MASPInput: N = %d, M = %d\n", m_N, m_M);

	// Dynamically allocate memory for I_ij
	I_ij = new bool*[m_N];
	for (int i = 0; i < m_N; ++i) {
		I_ij[i] = new bool[m_M];
	}

	// Initiate values
	for(int i = 0; i < m_N; i++) {
		for(int j = 0; j < m_M; j++) {
			I_ij[i][j] = false;
		}
	}
}

I_solution::I_solution(const I_solution &other) {
	// Initialize the size information
	m_N = other.m_N;
	m_M = other.m_M;
	m_input = other.m_input;

	// Dynamically allocate memory for I_ij
	I_ij = new bool*[m_N];
	for (int i = 0; i < m_N; ++i) {
		I_ij[i] = new bool[m_M];
		for(int j = 0 ; j < m_M ; j++){
			I_ij[i][j] = other.I_ij[i][j];
		}
	}
}

I_solution& I_solution::operator=(const I_solution &other) {
	// Initialize the size information
	m_N = other.m_N;
	m_M = other.m_M;
	m_input = other.m_input;

	// Dynamically allocate memory for I_ij
	I_ij = new bool*[m_N];
	for (int i = 0; i < m_N; ++i) {
		I_ij[i] = new bool[m_M];
		for(int j = 0 ; j < m_M ; j++){
			I_ij[i][j] = other.I_ij[i][j];
		}
	}

	return *this;
}

I_solution::~I_solution() {
	for (int i = 0; i < m_N; ++i) {
		delete[] I_ij[i];
	}
	delete[] I_ij;
}


// Prints this solution
void I_solution::PrintSolution() {
	printf("Solution I: N = %d, M = %d\n", m_N, m_M);

	// Print solution
	printf(" I_ij:\n");
	for (int i = 0; i < m_N; ++i) {
		for (int j = 0; j < m_M; ++j) {
			printf("  %d ", I_ij[i][j]);
		}
		printf("\n");
	}
}

// Assigns agent i to task j, updates i's position and traversed distance
void I_solution::Update(MASPInput* input, int i, int j) {
	// Need to update: I_ij, x_i, traversed-i
	if(DEBUG_I_SOL)
		printf("Assigning %d to %d\n", i, j);

	// Update I_ij
	for(int j = 0; j < m_M; j++) {
		// Reset all I_ij for this i
		I_ij[i][j] = false;
	}
	// Assign i to j
	I_ij[i][j] = true;
}

// Returns task j assigned to agent i
int I_solution::getTask(int i) {
	// Scan solution to find the agent assigned to i
	for(int j = 0; j < m_M; j++) {
		if(I_ij[i][j]) {
			// i is assigned to j
			return j;
		}
	}

	return -1;
}

// Returns the number of agents assigned to task j
int I_solution::getNj(int j) {
	int assigned_agents = 0;
	for(int i = 0; i < m_N; i++) {
		if(I_ij[i][j]) {
			assigned_agents++;
		}
	}

	return assigned_agents;
}

// Gets I_j by filling the given vector with the indices of the agents assigned to task j
void I_solution::getIj(std::vector<int>* I_j, int j) {
	for(int i = 0; i < m_N; i++) {
		if(I_ij[i][j]) {
			I_j->push_back(i);
		}
	}
}

/*
 * Determines the probability of mission success based on any assignment stored in
 * this solution the recursive ugly probability math. This function may take a long
 * time to run. This does not check to see that all agents are assigned to task or
 * that the agent-task assignments are compatible.
 */
double I_solution::BenchmarkRec() {
	// Probability that all tasks are complete
	double prob_success = 1;

	// Count the number of agents assigned to each task
	for(int j = 0; j < m_input->getM(); j++) {
		std::vector<int> I_j;
		int agent_count = 0;
		// Cycle through all agents
		for(int i = 0; i < m_input->getN(); i++) {
			if(I_ij[i][j]) {
				agent_count++;
				I_j.push_back(i);
			}
		}

		// Calculate the probability that d_j or more agents complete the task
		prob_success *= P_j(I_j, j);
	}

	// Sanity print
	if(DEBUG_I_SOL)
		printf("* p_a(I) = %f\n", prob_success);

	return prob_success;
}

/*
 * Determines the probability of mission success based on any assignment stored in
 * this solution using the closed-form Fourier transform of the ugly probability math.
 * This does not check to see that all agents are assigned to task or that the
 * agent-task assignments are compatible.
 */
double I_solution::BenchmarkClsdForm() {
	if(DEBUG_I_SOL)
		printf("Z = 0\n");

	double prob_success = 1;
	// For each task
	for(int j = 0; j < m_M; j++) {
		if(DEBUG_I_SOL)
			printf(" + {1");
		// Calculate the Poisson-binomial success function for k from d_j up to the number of agents assigned to j
		double Ps = 0;
		for(int k = m_input->get_d_j(j); k <= getNj(j); k++) {
			if(DEBUG_I_SOL)
				printf(" * P_f(K=%d, I_%d)", k, j);
			Ps += P_f(k, j);
		}
		if(DEBUG_I_SOL)
			printf("}");

		// Probability of complete success is the product of Ps(k, I_j) for k from d_j to n_j
		prob_success *= Ps;
	}
	if(DEBUG_I_SOL)
		printf(" = %f\n", prob_success);

	return prob_success;
}

/*
 * Estimates the probability of mission success based on any assignment stored in
 * this solution using Monte Carlo simulations. This does not check to see that all
 * agents are assigned to task or that the agent-task assignments are compatible.
 */
double I_solution::BenchmarkMonteCarlo() {
	double Z = 1;

	// Run through each task
	for(int j = 0; j < m_input->getM(); j++) {
		// Estimate the probability that j is completed
		Z *= monteSim(j);
	}

	return Z;
}

/*
 * Determines the average Z_i across all tasks. This is useful in larger inputs when
 * a single poor assignment can greatly reduce solution quality.
 */
double I_solution::AverageZi() {
	if(DEBUG_I_SOL)
		printf("\n");

	double sum_prob = 0;
	// For each task
	for(int j = 0; j < m_M; j++) {
		// Calculate the Poisson-binomial success function for k from d_j up to the number of agents assigned to j
		double Ps = 0;
		for(int k = m_input->get_d_j(j); k <= getNj(j); k++) {
			Ps += P_f(k, j);
		}

		// Probability of complete success is the product of Ps(k, I_j) for k from d_j to n_j
		sum_prob += Ps;
	}
	double avg_success = sum_prob/m_M;

	return avg_success;
}

// Returns the probability that exactly k agents assigned to task j will succeed
double I_solution::P_f(int k, int j) {
	double ret_val = 0.0;
	// How many agents are assigned to j?
	int n = 0;
	for(int i = 0; i < m_N; i++) {
		if(I_ij[i][j]) {
			n++;
		}
	}

	// Verify that this is possible...
	if(k > n) {
		// The probability that k > n agents succeed is zero...
		ret_val = 0.0;
	}
	else {
		// Assemble a list of probabilities that each agent is successful
		std::vector<double> p_values;
		for(int i = 0; i < m_N; i++) {
			if(I_ij[i][j]) {
				p_values.push_back(m_input->get_p_ij(i,j));
			}
		}

		// PMF - Probability of getting k successes out of n trials
		ret_val = poissonBinomialPMF(k, n, p_values);
	}

	return ret_val;
}


// Determines if this is a valid assignment solution (doesn't break constraints)
bool I_solution::ValidSolution() {
	/// Check each set of constraints
	bool valid = true;
	// Each agent is assigned at most once
	for(int i = 0; i < m_input->getN(); i++) {
		int count = 0;
		for(int j = 0; j < m_input->getM(); j++) {
			if(I_ij[i][j]) {
				count++;
			}
		}
		if(count > 1) {
			// Can only assign an agent at most one time
			valid = false;
		}
	}

	// Verify that each task has d_j or more agents
	for(int j = 0; j < m_input->getM(); j++) {
		int count = 0;
		for(int i = 0; i < m_input->getN(); i++) {
			if(I_ij[i][j]) {
				if(m_input->iCanDoj(i,j)) {
					count++;
				}
			}
		}

		// Did we find d_j or more?
		if(count < m_input->get_d_j(j)) {
			// No, not valid...
			valid = false;
		}
	}

	return valid;
}


/*
 * PMF of the Poisson-Binomial distribution - The probability of getting k successes out
 * of n trials, where each trial has probability given in p_values. This uses the
 * closed-form Fourier transform to calculate P(K=k).
 */
double I_solution::poissonBinomialPMF(int k, int n, std::vector<double>& p_values) {
	if((int64_t)n != (int64_t)p_values.size()) {
		// These should be the same... hard fail!
		fprintf(stderr, "[ERROR] : I_solution::poissonBinomialPMF : n = %d != |p_values| = %ld\n", n, p_values.size());
	}

	double ret_val = 0;

	// Handle by cases
	if(k == n) {
		// This is the product of all p_values
		ret_val = 1;
		for(double p : p_values) {
			ret_val *= p;
		}
	}
	else if(k == 0) {
		// This is the product of the inverse of all p_values
		ret_val = 1;
		for(double p : p_values) {
			ret_val *= (1-p);
		}
	}
	else {
		// Calculate the closed-form expression
		double factor = 1.0/(n+1);
		std::complex<double> outerSum = 0;
		if(DEBUG_I_SOL)
			printf("%f[ 0\n", factor);

		for(int l = 0; l <= n; l++) {
			// e^{i(2PI(-l)k)/(n+1)}
			std::complex<double> z1 = std::exp(1i * (2.0*PI*(-l)*k)/(n + 1.0));
			if(DEBUG_I_SOL)
				std::cout << " + (" << z1 << ")[ 1\n";

			std::complex<double> innerProd = 1;
			for(int m = 1; m <= n; m++) {
				// (1 + (e^{i(2PIl)/(n+1)} - 1)p_m)
				std::complex<double> z2 = std::exp(1i * (2.0*PI*l)/(n + 1.0));
				if(DEBUG_I_SOL)
					printf("   *(1 + (e^(%fi) - 1)%f)\n", (2.0*PI*l)/(n + 1.0), p_values.at(m-1));
				innerProd *= (1.0 + (z2 - 1.0)*p_values.at(m-1));
			}
			outerSum += z1*innerProd;

			if(DEBUG_I_SOL)
				printf("  ]\n");
		}
		ret_val = (factor*outerSum).real();

		if(DEBUG_I_SOL)
			std::cout << "] = " << (factor*outerSum) << '\n';
	}

	return ret_val;
}


// Calculates P_j for the agents in I_j
double I_solution::P_j(std::vector<int>& I_j, int j) {
	// Basic function parameteres
	int agent_count = static_cast<int>(I_j.size());
	double fP_j = 0;

	// Start with the probability that S_d_j agents can do the task, then increment
	for(int l = m_input->get_d_j(j), i = 0; l <= agent_count; l++, i++) {
		// Calculate S_l
		fP_j += pow(-1, i)*nChoosek(l-1, i)*S_l(I_j, j, l);
	}

	// Sanity print
	if(DEBUG_I_SOL)
		printf("P_%d(I_j) = ", j);
	for(int l = m_input->get_d_j(j), i = 0; l <= agent_count; l++, i++) {
		// Sanity print
		if(DEBUG_I_SOL)
			printf("(%f)%dS_%d + ", pow(-1, i), nChoosek(l-1, i), l);
	}
	if(DEBUG_I_SOL)
		printf("0 = %f\n", fP_j);

	return fP_j;
}

int I_solution::nChoosek(int n, int k) {
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
double I_solution::S_l(std::vector<int>& I_j, int j, int l) {
	double fS_l = 0;

	// Sanity print
	if(DEBUG_I_SOL) {
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
	if(DEBUG_I_SOL) {
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
			list_prob *= m_input->get_p_ij(i, j);
			// Sanity print
			if(DEBUG_I_SOL)
				printf("(%f)", m_input->get_p_ij(i, j));
		}
		fS_l += list_prob;
		// Sanity print
		if(DEBUG_I_SOL)
			printf(" + ");
	}

	// Sanity print
	if(DEBUG_I_SOL)
		printf("0\nS_%d(I_%d) = %f\n", l, j, fS_l);

	return fS_l;
}

// Compiles all combinations of length reqLen in list a
void I_solution::combination(int* a, int reqLen, int start, int currLen, bool* check, int len, std::vector<std::vector<int>>& combos) {
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

// Runs Monti Carlo Simulation to determine the probability that the agents in I_j complete task j
double I_solution::monteSim(int j) {
	double chance_of_success = 0;
	std::vector<int> I_j;
	for(int i = 0; i < m_input->getN(); i++) {
		if(I_ij[i][j]) {
			I_j.push_back(i);
		}
	}

	// Setup simulation
	int N = 1000;
	int total_successes = 0;

	// Run N simulations
	for(int l = 0; l < N; l++) {
		int run_successes = 0;
		// Cycle through each agent
		for(int i : I_j) {
			// Take a random outcome
			double rand_outcome = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			if(m_input->get_p_ij(i, j) > rand_outcome) {
				// Random outcome was within p_ij -> Agent succeeded!
				run_successes++;
			}
		}

		// Did enough agents succeed?
		if(run_successes >= m_input->get_d_j(j)) {
			// Enough agents succeeded, task was complete!
			total_successes++;
		}
	}
	chance_of_success = total_successes/double(N);

	return chance_of_success;
}
