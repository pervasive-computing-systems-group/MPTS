#include "PoissonBinomial.h"

PoissonBinomial::PoissonBinomial() {}

PoissonBinomial::~PoissonBinomial() {}

/*
 * PMF of the Poisson-Binomial distribution - The probability of getting k successes out
 * of n trials, where each trial has probability given in p_values. This uses the
 * closed-form Fourier transform to calculate P(K=k).
 */
double PoissonBinomial::PMF(int k, int n, std::vector<double>& p_values) {
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
		if(DEBUG_POISSONBINOMIAL)
			printf("%f[ 0\n", factor);

		for(int l = 0; l <= n; l++) {
			// e^{i(2PI(-l)k)/(n+1)}
			std::complex<double> z1 = std::exp(1i * (2.0*PI*(-l)*k)/(n + 1.0));
			if(DEBUG_POISSONBINOMIAL)
				std::cout << " + (" << z1 << ")[ 1\n";

			std::complex<double> innerProd = 1;
			for(int m = 1; m <= n; m++) {
				// (1 + (e^{i(2PIl)/(n+1)} - 1)p_m)
				std::complex<double> z2 = std::exp(1i * (2.0*PI*l)/(n + 1.0));
				if(DEBUG_POISSONBINOMIAL)
					printf("   *(1 + (e^(%fi) - 1)%f)\n", (2.0*PI*l)/(n + 1.0), p_values.at(m-1));
				innerProd *= (1.0 + (z2 - 1.0)*p_values.at(m-1));
			}
			outerSum += z1*innerProd;

			if(DEBUG_POISSONBINOMIAL)
				printf("  ]\n");
		}
		ret_val = (factor*outerSum).real();

		if(DEBUG_POISSONBINOMIAL)
			std::cout << "] = " << (factor*outerSum) << '\n';
	}

	return ret_val;
}

/*
 * Calculates the probability of success, which we define as the probability that d_j or
 * more trials are true where each trial has a probability of success given in p_values.
 * This uses the closed-form Fourier transform to calculate P(K=k).
 */
double PoissonBinomial::P_s(int d_j, int n, std::vector<double>& p_values) {
	// Calculate the Poisson-binomial success function for k from d_j up to n
	double Ps = 0;
	// For k in {d_j ... n_j}
	for(int k = d_j; k <= n; k++) {
		// Determine the probability that exactly k agents perform task j
		Ps += PMF(k, n, p_values);
	}

	return Ps;
}



//
//// Calculates P_j for the agents in I_j
//double I_solution::P_j(std::vector<int>& I_j, int j) {
//	// Basic function parameteres
//	int agent_count = static_cast<int>(I_j.size());
//	double fP_j = 0;
//
//	// Start with the probability that S_d_j agents can do the task, then increment
//	for(int l = m_input->get_d_j(j), i = 0; l <= agent_count; l++, i++) {
//		// Calculate S_l
//		fP_j += pow(-1, i)*nChoosek(l-1, i)*S_l(I_j, j, l);
//	}
//
//	// Sanity print
//	if(DEBUG_I_SOL)
//		printf("P_%d(I_j) = ", j);
//	for(int l = m_input->get_d_j(j), i = 0; l <= agent_count; l++, i++) {
//		// Sanity print
//		if(DEBUG_I_SOL)
//			printf("(%f)%dS_%d + ", pow(-1, i), nChoosek(l-1, i), l);
//	}
//	if(DEBUG_I_SOL)
//		printf("0 = %f\n", fP_j);
//
//	return fP_j;
//}
//
//int I_solution::nChoosek(int n, int k) {
//	if (k > n) return 0;
//	if (k * 2 > n) k = n-k;
//	if (k == 0) return 1;
//
//	int result = n;
//	for( int i = 2; i <= k; ++i ) {
//		result *= (n-i+1);
//		result /= i;
//	}
//
//	return result;
//}
//
//// Calculate S_l
//double I_solution::S_l(std::vector<int>& I_j, int j, int l) {
//	double fS_l = 0;
//
//	// Sanity print
//	if(DEBUG_I_SOL) {
//		printf("Finding S_%d for list: {", l);
//		for(int n : I_j) {
//			printf(" %d", n);
//		}
//		printf("}\n");
//	}
//
//	// Basic function setup
//	std::vector<std::vector<int>> combos;
//	int* arr = new int[I_j.size()];
//	bool* check = new bool[I_j.size()];
//
//	// Take the input of the array.
//	for(int i = 0; i < static_cast<int>(I_j.size()); i++) {
//		arr[i] = I_j.at(i);
//		check[i] = false;
//	}
//
//	// Compile a list of lists, each with size l and containing a
//	// unique combination of the elements in I_j
//	combination(arr, l, 0, 0, check, static_cast<int>(I_j.size()), combos);
//
//	// Sanity print
//	if(DEBUG_I_SOL) {
//		printf("Elements in combos:\n");
//		for(std::vector<int> list : combos) {
//			printf("{");
//			for(int n : list) {
//				printf(" %d", n);
//			}
//			printf("}");
//		}
//		puts("");
//	}
//
//	// Calculate S_l
//	for(std::vector<int> list : combos) {
//		double list_prob = 1;
//		for(int i : list) {
//			list_prob *= m_input->get_p_ij(i, j);
//			// Sanity print
//			if(DEBUG_I_SOL)
//				printf("(%f)", m_input->get_p_ij(i, j));
//		}
//		fS_l += list_prob;
//		// Sanity print
//		if(DEBUG_I_SOL)
//			printf(" + ");
//	}
//
//	// Sanity print
//	if(DEBUG_I_SOL)
//		printf("0\nS_%d(I_%d) = %f\n", l, j, fS_l);
//
//	return fS_l;
//}
//
//// Compiles all combinations of length reqLen in list a
//void I_solution::combination(int* a, int reqLen, int start, int currLen, bool* check, int len, std::vector<std::vector<int>>& combos) {
//	// Return if the currLen is more than the required length
//	if(currLen > reqLen) {
//		return;
//	}
//	else if (currLen == reqLen) {
//		// currLen is equal to required length -> record sequence
//		std::vector<int> tempList;
//		for (int i = 0; i < len; i++) {
//			if (check[i] == true) {
//				tempList.push_back(a[i]);
//			}
//		}
//
//		// Store this list of combinations
//		combos.push_back(tempList);
//		return;
//	}
//
//	// If start equals to len then return since no further element left.
//	if (start == len) {
//		return;
//	}
//
//	// For every index we have two options.
//	// First is, we select it, means put true in check[] and increment currLen and start.
//	check[start] = true;
//	combination(a, reqLen, start + 1, currLen + 1, check, len, combos);
//	// Second is, we don't select it, means put false in check[] and only start incremented.
//	check[start] = false;
//	combination(a, reqLen, start + 1, currLen, check, len, combos);
//}
//
//// Runs Monti Carlo Simulation to determine the probability that the agents in I_j complete task j
//double I_solution::monteSim(int j) {
//	double chance_of_success = 0;
//	std::vector<int> I_j;
//	for(int i = 0; i < m_input->getN(); i++) {
//		if(I_ij[i][j]) {
//			I_j.push_back(i);
//		}
//	}
//
//	// Setup simulation
//	int N = 1000;
//	int total_successes = 0;
//
//	// Run N simulations
//	for(int l = 0; l < N; l++) {
//		int run_successes = 0;
//		// Cycle through each agent
//		for(int i : I_j) {
//			// Take a random outcome
//			double rand_outcome = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
//			if(m_input->get_p_ij(i, j) > rand_outcome) {
//				// Random outcome was within p_ij -> Agent succeeded!
//				run_successes++;
//			}
//		}
//
//		// Did enough agents succeed?
//		if(run_successes >= m_input->get_d_j(j)) {
//			// Enough agents succeeded, task was complete!
//			total_successes++;
//		}
//	}
//	chance_of_success = total_successes/double(N);
//
//	return chance_of_success;
//}
