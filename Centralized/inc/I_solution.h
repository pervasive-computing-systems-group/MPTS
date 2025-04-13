/*
 * I_solution.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 7, 2023
 *
 * Description: Parent class for all solvers
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <complex>

#include "MASPInput.h"

#define DEBUG_I_SOL	DEBUG || 0

using namespace std::complex_literals;

class I_solution {
public:
	I_solution(MASPInput* input);
	virtual ~I_solution();
	I_solution(const I_solution &other);
	I_solution& operator=(const I_solution &other);

	// Prints this solution
	void PrintSolution();
	// Assigns agent i to task j
	void Update(MASPInput* input, int i, int j);
	// Returns task j assigned to agent i
	int getTask(int i);
	// Returns the number of agents assigned to task j
	int getNj(int j);
	// Gets I_j by filling the given vector with the indices of the agents assigned to task j
	void getIj(std::vector<int>* I_j, int j);
	/*
	 * Determines the probability of mission success based on any assignment stored in
	 * this solution the recursive ugly probability math. This function may take a long
	 * time to run. This does not check to see that all agents are assigned to task or
	 * that the agent-task assignments are compatible.
	 */
	double BenchmarkRec();
	/*
	 * Determines the probability of mission success based on any assignment stored in
	 * this solution using the closed-form Fourier transform of the ugly probability math.
	 * This does not check to see that all agents are assigned to task or that the
	 * agent-task assignments are compatible.
	 */
	double BenchmarkClsdForm();
	/*
	 * Estimates the probability of mission success based on any assignment stored in
	 * this solution using Monte Carlo simulations. This does not check to see that all
	 * agents are assigned to task or that the agent-task assignments are compatible.
	 */
	double BenchmarkMonteCarlo();
	/*
	 * Determines the average Z_i across all tasks. This is useful in larger inputs when
	 * a single poor assignment can greatly reduce solution quality.
	 */
	double AverageZi();
	// Returns the probability that exactly k agents assigned to task j will succeed
	double P_f(int k, int j);
	// Determines if this is a valid assignment solution (doesn't break constraints)
	bool ValidSolution();

	// Solution space - assigned tasks (NxM)
	bool** I_ij;
	// Number of agents
	int m_N;
	// Number of tasks
	int m_M;

private:
	/*
	 * PMF of the Poisson-Binomial distribution - The probability of getting k successes out
	 * of n trials, where each trial has probability given in p_values. This uses the
	 * closed-form Fourier transform to calculate P(K=k).
	 */
	double poissonBinomialPMF(int k, int n, std::vector<double>& p_values);
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

	MASPInput* m_input;
};
