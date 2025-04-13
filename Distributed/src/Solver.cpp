#include "Solver.h"

Solver::Solver(int nodeID, std::string input_path) : sInput(input_path) {
	// Log our start time
	startTime = std::chrono::high_resolution_clock::now();

/*
  This function will read in an input file for the MAS Problem. Lines
  that start with the '#' symbol will be ignored.
  Expected file structure:
	N M E
	c_ik - N x E
	 ...
	r_jk - M x E
	 ...
	p_ij - N x M
	 ...
	d_j  - M
	 ...
 */

	// Initial assignment, silence annoying macro warnings
	// and avoid issues if parsing fails
	M = 0;
	E = 0;
	c_k = NULL;
	r_jk = NULL;
	p_j = NULL;
	d_j = NULL;

	if(SANITY_PRINT)
		printf("[SLVR] Reading MASP input\n");

	// Read status
	bool read_success = true;

	// Open file
	std::ifstream file(input_path);
	std::string line;
	// Grab first line, should have N, M, and  E
	if(getNextLine(&file, &line)) {
		std::stringstream lineStreamNEM(line);
		// Push values into N, M, E
		lineStreamNEM >> N;
		lineStreamNEM >> M;
		lineStreamNEM >> E;
	}
	else {
		// Line reading failed
		read_success = false;
	}
	// Sanity print
	if(SANITY_PRINT)
		printf("[SLVR]  N = %d, M = %d, E = %d\n", N, M, E);

	// Read in agent capabilities
	c_k = new int[E];
	// Sanity print
	if(DEBUG_SOLVER)
		printf(" c_ik: ");
	for(int i = 0; i < N; i++) {
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_c(line);
			// Expected E values for each c_ik[i]
			if(i == nodeID) {
				// This is our agent... pull in the capabilities data
				for(int k = 0; k < E; k++) {
					lineStream_c >> c_k[k];
					// Sanity print
					if(DEBUG_SOLVER)
						printf("%d ", c_k[k]);
				}
			}
			// Else... do nothing with the line
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_SOLVER)
		puts("");

	// Read in role requirements
	r_jk = new int*[M];
	// Sanity print
	if(DEBUG_SOLVER)
		printf(" r_jk:");
	for(int j = 0; j < M; j++) {
		r_jk[j] = new int[E];
		// Sanity print
		if(DEBUG_SOLVER)
			printf("\n  %d:\t", j);
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_r(line);
			// Expected E values for each r_jk[i]
			for(int k = 0; k < E; k++) {
				lineStream_r >> r_jk[j][k];
				// Sanity print
				if(DEBUG_SOLVER)
					printf("%d\t", r_jk[j][k]);
			}
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_SOLVER)
		puts("");

	// Read in robot-to-role costs
	p_j = new double[M];
	// Sanity print
	if(DEBUG_SOLVER)
		printf(" p_ij: ");
	for(int i = 0; i < N; i++) {
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_d(line);
			if(i == nodeID) {
				// Expected N values for each d_ij[i]
				for(int j = 0; j < M; j++) {
					lineStream_d >> p_j[j];
					// Sanity print
					if(DEBUG_SOLVER)
						printf("%.2f\t", p_j[j]);
				}
			}
			// Else... do nothing
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_SOLVER)
		puts("");

	// Read in robot-to-role requirements
	d_j = new int[M];
	// Sanity print
	if(DEBUG_SOLVER)
		printf(" d_j:");
	for(int j = 0; j < M; j++) {
		// Sanity print
		if(DEBUG_SOLVER)
			printf("\n  %d:\t", j);
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_d(line);
			// Expect a single value for d_j
			lineStream_d >> d_j[j];
			// Sanity print
			if(DEBUG_SOLVER)
				printf("%d\t", d_j[j]);
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_SOLVER)
		puts("");

	// Verify that we successfully read the input file
	if(!read_success) {
		// Input file not formatted correctly, hard fail!
		fprintf(stderr, "[MASPInput::MASPInput] : Input file format off\n");
		exit(1);
	}
	else {
		if(SANITY_PRINT)
			printf("[SLVR] Successfully read input\n\n");
	}
}

Solver::~Solver() {}


// Gets the next valid line from file, stores it in line. Will ignore
// lines that start with '#' symbol.
bool Solver::getNextLine(std::ifstream* file, std::string* line) {
	bool run_again = true;
	bool read_success = false;
	// Grab next line un-commented line
	while(run_again) {
		if(std::getline(*file, *line)) {
			if(0) {
				printf("Next line: \"%s\"\n", (*line).c_str());
				printf("First char: \'%d\'\n", (*line)[0]);
			}
			// Successfully read next line, verify it doesn't start with '#'
			if((*line)[0] != '#') {
				// Found next valid input line
				run_again = false;
				read_success = true;
			}
			else {
				// Line starts with '#', get next line
				run_again = true;
				if(0)
					puts("Ignoring line");
			}
		}
		else {
			// Failed to read next line
			run_again = false;
			read_success = false;
		}
	}

	if(0)
		printf("Return Line: \"%s\"\n", (*line).c_str());

	return read_success;
}

// Determines if i is capable of performing j
bool Solver::iCanDoj(int j) {
	if(j < M) {
		// Iterate through capabilities
		for(int k = 0; k < E; k++) {
			// Check compatibility for capability k
			if(c_k[k] - r_jk[j][k] < 0) {
				// Incompatible, end here
				return false;
			}
		}
		// If we made it this far, i can do j
		return true;
	}
	else {
		// i or j is out of bounds... You're doing something work -> hard fail!
		fprintf(stderr, "[Solver::iCanDoj] : Asked for bad i/j combo\n");
		exit(1);
	}
}

/*
 * Determines the probability of mission success given the solution assignment
 * in x_ij. Assumes that any non-zero entry in x_ij means that agent i has
 * been assigned to task j.
 */
double Solver::BenchmarkCF(agent_tp* agent_table) {
	// Probability that all tasks are complete
	double prob_success = 1;

	// Create a double vector of tasks
	std::vector<std::vector<double>> p_values;
	for(int j = 0; j < M; j++) {
		std::vector<double> temp;
		p_values.push_back(temp);
	}

	// For each agent
	for(int i = 0; i < N; i++) {
		// Push i's entry into j (if j isn't -1)
		if(agent_table[i].task >= 0) {
			p_values.at(agent_table[i].task).push_back(agent_table[i].probability);
		}
	}

	// For each task
	for(int j = 0; j < M; j++) {

		// Calculate the Poisson-binomial success function for k from d_j up to the number of agents assigned to j
		double Ps = 0;
		long int N_j = p_values.at(j).size();
		// For k in {d_j ... n_j}
		for(int k = d_j[j]; k <= N_j; k++) {
			// Determine the probability that exactly k agents perform task j
			Ps += m_poissonBinomial.PMF(k, N_j, p_values.at(j));
		}

		// Probability of complete success is the product of Ps(k, I_j) for k from d_j to n_j
		prob_success *= Ps;

		// If we hit zero... just give up
		if(isZero(prob_success)) {
			break;
		}
	}

	return prob_success;
}

// Returns the probability that this node performs j
double Solver::get_p_j(int j) {
	double retVal = p_j[j];

	// Check if the agent meets the requirements of j
	for(int k = 0; k < E; k++) {
		// Do we meet the requirements?
		if( (c_k[k]-r_jk[j][k]) < 0 ) {
			// No, we don't...
			retVal = 0.0;
		}
	}

	return retVal;
}

