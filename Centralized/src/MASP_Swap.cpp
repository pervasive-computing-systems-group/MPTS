#include "MASP_Swap.h"


MASP_Swap::MASP_Swap() {
	if(SANITY_PRINT)
		printf("Hello from MASP_Swap Solver!\n");
	srand (time(NULL));
	a_r=0;
	a_f=0;
	np=0;
}


void MASP_Swap::Solve(MASPInput* input, I_solution* I_crnt) {
	/*
	 * Treat problem as bijection matching problem
	 *
	 * costs
	 *    1.1 1.2 1.x 2.1 2.2 2.3 2.x :tasks
	 * 1  q11 q11 q11 q12 q12
	 * 2  ...
	 * 3
	 * 4
	 * 5
	 * 6
	 * 7'  1   1   0
	 * :agents
	 *
	 * q_ij = (1-p_ij)
	 */

	// Determine a_r, the number of required agents
	std::string input_file = input->input_fileName;	
	a_r = 0;
	for(int j = 0; j < input->getM(); j++) {
		a_r += input->get_d_j(j);
	}
	// Determine a_f, the number of floating agents
	a_f = input->getN() - a_r;
	// Determine np, the number of participants for balanced matching
	np = a_r + input->getM()*a_f;
	if(DEBUG_MASP_SWAP) {
		printf("n=%d, m=%d, a_r=%d, a_f=%d, np=%d\nTask Row:\n", input->getN(), input->getM(), a_r, a_f, np);
		for(int j = 0; j < input->getM();j++) {
			for(int i = 0; i < input->get_d_j(j); i++) {
				printf(" %d.%d", j, i);
			}
			for(int i = 0; i < a_f; i++) {
				printf(" %d.x", j);
			}
		}
		printf("\n");
	}

	// Create a cost map
	std::vector<std::vector<double> > costMatrix;
	for(int i = 0; i < np; i ++) {
		std::vector<double> temp;

		// Is this a real agent?
		int agentI = get_agent(i, input);
		if(agentI >= 0) {
			for(int j = 0; j < np; j++) {
				int taskJ = get_task(j, input);
				// Can i do j?
				if(input->iCanDoj(agentI, taskJ)) {
					temp.push_back(1-input->get_p_ij(agentI, taskJ));
				}
				else {
					// i can't do j... assign INF
					temp.push_back(INF);
				}
			}
		}
		else {
			// Phantom agent.. assign 1 for non-floating task and 0 for floating tasks
			for(int j = 0; j < np; j++) {
				if(floating_task(j, input)) {
					// Phantom agents prefer floating tasks
					temp.push_back(0);
				}
				else {
					// Phantoms do not prefer real tasks
					temp.push_back(INF);
				}
			}
		}
		costMatrix.push_back(temp);
	}

	// Sanity print
	if(DEBUG_MASP_SWAP) {
		printf("Cost map:\n");
		for(int j = 0; j < np; j ++) {
			for(int i = 0; i < np; i++) {
				printf(" %.2f", costMatrix.at(i).at(j));
			}
			puts("");
		}
	}

	vector<int> assignment;
	mHungAlgo.Solve(costMatrix,assignment);

	// Sanity print
	if(DEBUG_MASP_SWAP) {
		printf("Matchings:\n");
		for(long unsigned int i = 0; i < assignment.size(); i++) {
			printf(" %ld:%d\n", i, assignment.at(i));
		}
	}

	for(int i = 0; i < input->getN(); i++) {
		I_crnt->Update(input,i, get_task(assignment.at(i), input));
	}

	if(DEBUG_MASP_SWAP){
		I_crnt->PrintSolution();
    }
	
	// plot_40_36 is a plot that shows 4 swap working
	// plot 20_10 shows 3 swap working
	// printf("***---Begin Swap Checking---***\n");
	double before_swap_benchmark = I_crnt->BenchmarkClsdForm();
	I_solution solnArr[] = {I_solution(*I_crnt), I_solution(*I_crnt), I_solution(*I_crnt), I_solution(*I_crnt), I_solution(*I_crnt)};
	solnArr[0] = *I_crnt;
	I_solution outputHolder{*I_crnt};
	double benchmarks[] = {before_swap_benchmark, 0.0, 0.0, 0.0, 0.0};

	// printf("Search for beneficial swaps with single agents...\n");
	// single_swap(I_crnt, input, outputHolder);
	// benchmarks[1] = outputHolder.BenchmarkClsdForm();
	// solnArr[1] = outputHolder;
	// outputHolder = *I_crnt;

	// printf("Search for beneficial swaps with 2 agents...\n");
	two_swap(I_crnt, input, outputHolder);
	benchmarks[1] = outputHolder.BenchmarkClsdForm();
	solnArr[1] = outputHolder;
	outputHolder = *I_crnt;

	// printf("Search for beneficial swaps with 3 agents...\n");
	three_swap(I_crnt, input, outputHolder);
	benchmarks[2] = outputHolder.BenchmarkClsdForm();
	solnArr[2] = outputHolder;
	outputHolder = *I_crnt;

	// printf("Search for beneficial swaps with 4 agents...\n");
	four_swap(I_crnt, input, outputHolder);
	benchmarks[3] = outputHolder.BenchmarkClsdForm();
	solnArr[3] = outputHolder;

	// printf("Original solution has value of %f\n", before_swap_benchmark);
	// solnArr[0].PrintSolution();
	double best = before_swap_benchmark;
	int bestIndex = 0;
	for(int i = 1 ; i < 4; i++){
		// printf("Swap %d solution has value of %f\n", i, benchmarks[i]);
		// solnArr[i].PrintSolution();
		if(benchmarks[i] >= best){
			best = benchmarks[i];
			bestIndex = i;
			*I_crnt = solnArr[i];
		}
	}
	printf("Best swap solution is swap %d with value of %f as compared to the original value which was%f\n", bestIndex, best, before_swap_benchmark);
	std::ofstream outfile("Swapresults.txt", std::ios::app);
	outfile << best << std::endl;
	// outfile << "Input file: " << input_file << " has the following results: " << "Best Swap soln is swap " << bestIndex << " with value of " << best << " as compared to the original value which was " << before_swap_benchmark << 
	//  " with a difference of " << best - before_swap_benchmark << std::endl;
;
}

void MASP_Swap::single_swap(I_solution* I_crnt, MASPInput* input, I_solution& bestSoln){
	double baseBenchMark = I_crnt->BenchmarkClsdForm();
	I_solution newPossibleSoln(*I_crnt);
	for(int i = 0 ; i < I_crnt->m_N; i++){
		int currTask = I_crnt->getTask(i);
		for(int j = 0 ; j < I_crnt->m_M; j++){
			if(currTask == j){
				continue;
			}
			if(input->iCanDoj(i,j)){
				newPossibleSoln.Update(input, i, j);
				if(newPossibleSoln.ValidSolution()){
					double newPossibleSolnBenchmark = newPossibleSoln.BenchmarkClsdForm();
					if(newPossibleSolnBenchmark > baseBenchMark){
						baseBenchMark = newPossibleSolnBenchmark;
						bestSoln = newPossibleSoln;
					}
				}
				newPossibleSoln.Update(input, i, currTask);
			}
		}
	}
	// delete &newPossibleSoln;
}

void MASP_Swap::two_swap(I_solution* I_crnt, MASPInput* input, I_solution& bestSoln){
	I_solution newPossibleSoln(*I_crnt);
	for(int i = 0 ; i < I_crnt->m_N; i++){
		for(int j = i + 1 ; j < I_crnt->m_N; j++){
			int currAgentTask = I_crnt->getTask(i);
			int potentialSwapAgentTask = I_crnt->getTask(j);
			if(currAgentTask == potentialSwapAgentTask){ //agents are already assigned to the same task and cant be swapped
				continue;
			}
			double currentSolnBenchmark = bestSoln.BenchmarkClsdForm();
			bool iCanDoJ = input->iCanDoj(i,potentialSwapAgentTask); //swapping jobs
			bool jCanDoI = input->iCanDoj(j,currAgentTask); //swapping jobs
			if(iCanDoJ && jCanDoI){
				newPossibleSoln.Update(input, i, potentialSwapAgentTask);
				newPossibleSoln.Update(input, j, currAgentTask);
				if(newPossibleSoln.ValidSolution()){
					double newPossibleSolnBenchmark = newPossibleSoln.BenchmarkClsdForm();
					if(newPossibleSolnBenchmark > currentSolnBenchmark){ //new solution is better than the old solution. Save new solution as the best and continue to the next iteration
						bestSoln = newPossibleSoln;
					}
					newPossibleSoln.Update(input, i, currAgentTask);
					newPossibleSoln.Update(input, j, potentialSwapAgentTask);
				}
			}
			else{ //job swap is not possible because of agent abilities
			}
		}
	}
	// delete &newPossibleSoln;
}

void MASP_Swap::three_swap(I_solution* I_crnt, MASPInput* input, I_solution& bestSoln){ //312 and 231 are only combos to check
	I_solution newPossibleSolnOne(*I_crnt);
	I_solution newPossibleSolnTwo(*I_crnt);
	for(int i = 0 ; i < I_crnt->m_N; i++){
		for(int j = i + 1 ; j < I_crnt->m_N; j++){
			for(int k = j + 1 ; k < I_crnt->m_N ; k++){
				int ithTask = I_crnt->getTask(i);
				int jthTask = I_crnt->getTask(j);
				int kthTask = I_crnt->getTask(k);
				double currentSolnBenchmark = bestSoln.BenchmarkClsdForm();
				//check that job swaps are possible for all possible job swaps
				bool iCanDoJ = input->iCanDoj(i,jthTask);
				bool iCanDoK = input->iCanDoj(i,kthTask);
				bool jCanDoI = input->iCanDoj(j,ithTask);
				bool jCanDoK = input->iCanDoj(j,kthTask);
				bool kCanDoI = input->iCanDoj(k,ithTask);
				bool kCanDoJ = input->iCanDoj(k,jthTask);

				if(iCanDoJ && iCanDoK && jCanDoI && jCanDoK && kCanDoI && kCanDoJ){ //312 is k->1, i->2, j->3
					newPossibleSolnOne.Update(input, i, jthTask);
					newPossibleSolnOne.Update(input, j, kthTask);
					newPossibleSolnOne.Update(input, k, ithTask);
					double benchmarkOne = 0.0;
					if(newPossibleSolnOne.ValidSolution()){
						benchmarkOne = newPossibleSolnOne.BenchmarkClsdForm();
					}
					
					newPossibleSolnTwo.Update(input, i, kthTask); // 231 is j->1, k->2, i->3
					newPossibleSolnTwo.Update(input, j, ithTask);
					newPossibleSolnTwo.Update(input, k, jthTask);
					double benchmarkTwo = 0.0;
					if(newPossibleSolnTwo.ValidSolution()){
						benchmarkTwo = newPossibleSolnTwo.BenchmarkClsdForm();
					}
					
					if(benchmarkOne > benchmarkTwo && benchmarkOne > currentSolnBenchmark){
						bestSoln = newPossibleSolnOne;
					}
					else if(benchmarkTwo > benchmarkOne && benchmarkTwo > currentSolnBenchmark){
						bestSoln = newPossibleSolnTwo;
					}
					//reset both potential solns back to their original states before next iter of for loop
					newPossibleSolnOne.Update(input, i, ithTask);
					newPossibleSolnOne.Update(input, j, jthTask);
					newPossibleSolnOne.Update(input, k, kthTask);
					newPossibleSolnTwo.Update(input, i, ithTask);
					newPossibleSolnTwo.Update(input, j, jthTask);
					newPossibleSolnTwo.Update(input, k, kthTask);
				}
			}
		}
	}
	// delete &newPossibleSolnOne;
	// delete &newPossibleSolnTwo;
}
// Looking for possible swaps for jobs for 4 agents where all agents are assigned to a different task (not something like: 4213 where agent 2 would not 
// be assigned to a differnt job): 4213 would be the same as a 3 swap for agents 4,1 and 3.
// There are 9 assignments possible for 4 agents: 2143, 2341, 2413, 3142, 3421, 3412, 4123, 4312, 4321
void MASP_Swap::four_swap(I_solution* I_crnt, MASPInput* input, I_solution& bestSoln){
	I_solution newPossibleSolnOne(*I_crnt);
	for(int i = 0 ; i < I_crnt->m_N; i++){
		for(int j = i + 1 ; j < I_crnt->m_N; j++){
			for(int k = j + 1 ; k < I_crnt->m_N ; k++){
				for(int l = k + 1 ; l < I_crnt->m_N ; l++){
					int ithTask = I_crnt->getTask(i);
					int jthTask = I_crnt->getTask(j);
					int kthTask = I_crnt->getTask(k);
					int lthTask = I_crnt->getTask(l);
					double currentSolnBenchmark = bestSoln.BenchmarkClsdForm();
					//check that job swaps are possible for all possible job swaps (12 checks in total)
					bool iCanDoJ = input->iCanDoj(i,jthTask);
					bool iCanDoK = input->iCanDoj(i,kthTask);
					bool iCanDoL = input->iCanDoj(i,lthTask);
					bool jCanDoI = input->iCanDoj(j,ithTask);
					bool jCanDoK = input->iCanDoj(j,kthTask);
					bool jCanDoL = input->iCanDoj(j,lthTask);
					bool kCanDoI = input->iCanDoj(k,ithTask);
					bool kCanDoJ = input->iCanDoj(k,jthTask);
					bool kCanDoL = input->iCanDoj(k,lthTask);
					bool lCanDoI = input->iCanDoj(l,ithTask);
					bool lCanDoJ = input->iCanDoj(l,jthTask);
					bool lCanDoK = input->iCanDoj(l,kthTask);

					double* bestBenchmarkHolder = new double;
					*bestBenchmarkHolder = currentSolnBenchmark;

					bool allJobsPossible = iCanDoJ && iCanDoK && iCanDoL && jCanDoI && jCanDoK && jCanDoL && kCanDoI
					 && kCanDoJ && kCanDoL && lCanDoI && lCanDoJ && lCanDoK;

					if(allJobsPossible){ // all job swaps possible
						// check swap 2143 (i->2, j->1, k->4, l->3)
						newPossibleSolnOne.Update(input, i, jthTask);
						newPossibleSolnOne.Update(input, j, ithTask);
						newPossibleSolnOne.Update(input, k, lthTask);
						newPossibleSolnOne.Update(input, l, kthTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}
						// check swap 2341 (i->2, j->3, k->4, l->1)
						newPossibleSolnOne.Update(input, i, jthTask);
						newPossibleSolnOne.Update(input, j, kthTask);
						newPossibleSolnOne.Update(input, k, lthTask);
						newPossibleSolnOne.Update(input, l, ithTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}
						// check swap 2413 (i->2, j->4, k->1, l->3)
						newPossibleSolnOne.Update(input, i, jthTask);
						newPossibleSolnOne.Update(input, j, lthTask);
						newPossibleSolnOne.Update(input, k, ithTask);
						newPossibleSolnOne.Update(input, l, kthTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}

						// check swap 3142 (i->3, j->1, k->4, l->2)
						newPossibleSolnOne.Update(input, i, kthTask);
						newPossibleSolnOne.Update(input, j, ithTask);
						newPossibleSolnOne.Update(input, k, lthTask);
						newPossibleSolnOne.Update(input, l, jthTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}

						// check swap 3421 (i->3, j->4, k->2, l->1)
						newPossibleSolnOne.Update(input, i, kthTask);
						newPossibleSolnOne.Update(input, j, lthTask);
						newPossibleSolnOne.Update(input, k, jthTask);
						newPossibleSolnOne.Update(input, l, ithTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}

						// check swap 3412 (i->3, j->4, k->1, l->2)
						newPossibleSolnOne.Update(input, i, kthTask);
						newPossibleSolnOne.Update(input, j, lthTask);
						newPossibleSolnOne.Update(input, k, ithTask);
						newPossibleSolnOne.Update(input, l, jthTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}

						// check swap 4123 (i->4, j->1, k->2, l->3)
						newPossibleSolnOne.Update(input, i, lthTask);
						newPossibleSolnOne.Update(input, j, ithTask);
						newPossibleSolnOne.Update(input, k, jthTask);
						newPossibleSolnOne.Update(input, l, kthTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}

						// check swap 4312 (i->4, j->3, k->1, l->2)
						newPossibleSolnOne.Update(input, i, lthTask);
						newPossibleSolnOne.Update(input, j, kthTask);
						newPossibleSolnOne.Update(input, k, ithTask);
						newPossibleSolnOne.Update(input, l, jthTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}

						// check swap 4321 (i->4, j->3, k->2, l->1)
						newPossibleSolnOne.Update(input, i, lthTask);
						newPossibleSolnOne.Update(input, j, kthTask);
						newPossibleSolnOne.Update(input, k, jthTask);
						newPossibleSolnOne.Update(input, l, ithTask);

						if(newPossibleSolnOne.ValidSolution()){
							double newBenchmark = newPossibleSolnOne.BenchmarkClsdForm();
							if(*bestBenchmarkHolder < newBenchmark){
								*bestBenchmarkHolder = newBenchmark;
								bestSoln = newPossibleSolnOne;
							}
						}
						//reset potential soln back to their original states before next iter of for loop
						newPossibleSolnOne.Update(input, i, ithTask);
						newPossibleSolnOne.Update(input, j, jthTask);
						newPossibleSolnOne.Update(input, k, kthTask);
						newPossibleSolnOne.Update(input, l, lthTask);
					}
				}
			}
		}
	}
	// delete &newPossibleSolnOne;
}

// Takes balanced matching index and converts it into the corresponding task's index
int MASP_Swap::get_task(int bal_j, MASPInput* input) {
	int run_j = 0;
	for(int j = 0; j < input->getM();j++) {
		for(int i = 0; i < input->get_d_j(j); i++) {
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
	fprintf(stderr, "[MASP_Swap::get_task] : Could not find mapping from bal_j to task_j, bal_j = %d\n", bal_j);
	exit(1);
}

// Takes balanced matching index and converts it into the corresponding agent's index.
// Returns -1 if this is a phantom
int MASP_Swap::get_agent(int i, MASPInput* input) {
	if(i < input->getN()) {
		return i;
	}

	// This is a phantom women
	return -1;
}

// Determines if this is a floating task
bool MASP_Swap::floating_task(int bal_j, MASPInput* input) {
	int run_j = 0;
	for(int j = 0; j < input->getM();j++) {
		for(int i = 0; i < input->get_d_j(j); i++) {
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
	fprintf(stderr, "[MASP_Swap::floating_task] : Could not find mapping from bal_j to task_j, bal_j = %d\n", bal_j);
	exit(1);
}

int MASP_Swap::resolve_task(int jj, MASPInput* input) {
	int j = 0;
	int j_track = 0;

	while(j < input->getM()) {
		if(jj >= j_track && jj < (j_track+input->get_d_j(j))) {
			return j;
		}

		// Update j and j-tracker
		j_track += input->get_d_j(j);
		j++;
	}

	// This must be a floating task
	return -1;
}
