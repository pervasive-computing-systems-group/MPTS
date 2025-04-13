#include <stdio.h>
#include <vector>
#include <limits>
#include <chrono>

#include "defines.h"
#include "MASPInput.h"
#include "MASP_comp.h"
#include "MASP_FastComp.h"
#include "MASP_BalMatch.h"
#include "MASP_TMatch.h"
#include "I_solution.h"
#include "MASP_MatchAct.h"
#include "MASP_EdgeCutting.h"
#include "MASP_GradientSearch.h"
#include "MASP_MatchGS.h"
#include "MASP_MinDist.h"
#include "MASP_LogBalMatch.h"
#include "MASP_Swap.h"
#include "MASP_BranchAndMatch.h"
#include "MASP_BranchAndBound.h"



#define DEBUG_MAIN	DEBUG || 0

#define REC_COMP_Z		0
#define ESTIMATE_Z		1
#define PRINT_RESULTS	0
#define DATA_LOG_FORMAT	"alg_%d.dat"
#define DATA_LOG_DEFLT_PATH	""


int main(int argc, char *argv[]) {
	srand(time(NULL));

	int algorithm = 3;
	bool printResults;
	const char* outputPath;
	int runnum = 0;

	// Verify user input
	if(argc == 3) {
		algorithm = atoi(argv[2]);
		printResults = PRINT_RESULTS;
		outputPath = DATA_LOG_DEFLT_PATH;
	}
	else if(argc == 4) {
		algorithm = atoi(argv[2]);
		printResults = atoi(argv[3]);
		outputPath = DATA_LOG_DEFLT_PATH;
	}
	else if(argc == 5) {
		algorithm = atoi(argv[2]);
		printResults = atoi(argv[3]);
		outputPath = argv[4];
	}
	else if(argc == 6) {
		algorithm = atoi(argv[2]);
		printResults = atoi(argv[3]);
		outputPath = argv[4];
		runnum = atoi(argv[5]);
	}
	else {
		printf("Received %d args, expected 1 or more.\nExpected use:\t./find-assignment <file path> [algorithm] [print results] [output path] [run number]\n\n", argc - 1);
		return 1;
	}

	Solver* solver = NULL;
	MASPInput input(argv[1]);
	I_solution solution(&input);

	// Capture start time
	auto start = std::chrono::high_resolution_clock::now();

	switch(algorithm) {
	case e_Algo_MASP_COMP: {
		solver = new MASPComp();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_FAST_COMP: {
		solver = new MASP_FastComp();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_TMTCH: {
		solver = new MASP_TMatch();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_BMTCH: {
		solver = new MASP_BalMatch();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_MTCHACT: {
		solver = new MASP_MatchAct();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_EDGCUT: {
		solver = new MASP_EdgeCutting();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_GRADSEARCH: {
		solver = new MASP_GradientSearch();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_MTCHGS: {
		solver = new MASP_MatchGS();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_MIN_DIST: {
		solver = new MASP_MinDist();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_LOGMATCH: {
		solver = new MASP_LogBalMatch();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_SWAP: {
		solver = new MASP_Swap();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_BNM: {
		solver = new MASP_BranchAndMatch();
		solver->Solve(&input, &solution);
	}
	break;

	case e_Algo_MASP_BNB: {
		solver = new MASP_BranchAndBound();
		solver->Solve(&input, &solution);
	}
	break;

	default:
		// No valid algorithm given
		fprintf(stderr, "[ERROR][main] : \n\tInvalid algorithm identifier!\n");
		exit(1);

	}

	// Capture end time
	auto stop = std::chrono::high_resolution_clock::now();
	// Determine the time it took to solve this
	long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	double duration_s = (double)duration/1000.0;

	// Results
	double closed_Z = solution.BenchmarkClsdForm();
	double upperBound = input.UpperBound();
	double computed_Z = 0;
	double estimated_Z = 0;
	double average_Z_i = solution.AverageZi();

	if(SANITY_PRINT) {
		printf("\nUpper bound on solution: %f\n", upperBound);
		printf("Closed-form Z = %f\n", closed_Z);
		printf("Average Z_i = %f\n", average_Z_i);
	}

	// Determine how well this solution worksoutputPath
	if(REC_COMP_Z) {
		computed_Z = solution.BenchmarkRec();
		if(SANITY_PRINT)
			printf("Computed Z = %f\n", computed_Z);
	}
	if(ESTIMATE_Z)  {
		// Run Monte Carlo simulations to estimate Z
		estimated_Z = solution.BenchmarkMonteCarlo();
		if(SANITY_PRINT)
			printf("Estimated Z = %f\n", estimated_Z);
	}

	if(SANITY_PRINT)
		printf("Computation time = %f s\n", duration_s);

	// Print results to file
	if(printResults) {
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, "%s", outputPath);
		sprintf(buff + strlen(buff), DATA_LOG_FORMAT, algorithm);
		if(SANITY_PRINT)
			printf(" Printing results to: %s\n", buff);
		pOutputFile = fopen(buff, "a");
		// File format: n m runmun computed_Z estimated_Z comp-time
		fprintf(pOutputFile, "%d %d %d ", input.getN(), input.getM(), runnum);
		fprintf(pOutputFile, "%.10f %.10f %.10f %f", closed_Z, estimated_Z, upperBound, duration_s);
		// If this does gradient search, print the number of times the algorithm iterated
		if(algorithm == e_Algo_MASP_GRADSEARCH) {
			fprintf(pOutputFile, " %d", ((MASP_GradientSearch*)solver)->getIterations());
		}
		else if(algorithm == e_Algo_MASP_MTCHGS) {
			fprintf(pOutputFile, " %d", ((MASP_MatchGS*)solver)->getIterations());
		}
		fprintf(pOutputFile, " %d", solution.ValidSolution());
		fprintf(pOutputFile, " %f\n", average_Z_i);
		fclose(pOutputFile);
	}

	delete solver;

	if(SANITY_PRINT)
		printf("Done\n");
}
