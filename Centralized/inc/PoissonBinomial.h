/*
 * PoissonBinomial.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jan 05, 2024
 *
 * Description: Class that handles math related to a Poisson-Binomial distribution
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <complex>
#include <sstream>
#include <fstream>

#include "defines.h"

#define DEBUG_POISSONBINOMIAL	DEBUG || 0

using namespace std::complex_literals;

class PoissonBinomial {
public:
	PoissonBinomial();
	~PoissonBinomial();

	/*
	 * PMF of the Poisson-Binomial distribution - The probability of getting k successes out
	 * of n trials, where each trial has probability given in p_values. This uses the
	 * closed-form Fourier transform to calculate P(K=k).
	 */
	double PMF(int k, int n, std::vector<double>& p_values);

	/*
	 * Calculates the probability of success, which we define as the probability that d_j or
	 * more trials are true where each trial has a probability of success given in p_values.
	 * This uses the closed-form Fourier transform to calculate P(K=k).
	 */
	double P_s(int d_j, int n, std::vector<double>& p_values);

private:
	// Calculates P_j for the agents in I_j
	double P_j(std::vector<int>& I_j, int j);
	// Calculates nCk
	int nChoosek(int n, int k);
	// Calculate S_l
	double S_l(std::vector<int>& I_j, int j, int l);
	// Compiles all combinations of length reqLen in list a
	void combination(int* a, int reqLen, int start, int currLen, bool* check, int len, std::vector<std::vector<int>>& combos);
	// Runs Monti Carlo Simulation to determine the probability that the agents in I_j complete task j
	double monteSim(int j);
};
