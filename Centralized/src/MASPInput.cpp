#include "MASPInput.h"

/*
 * MASPInput Constructor. Takes in an input file path. The MAS Problem has the
 * following problem structure:
 *
 * Sets:
 * i in I, set of agents, |I| = N
 * j in J, set of tasks, |J| = M
 * k in K, set of capabilities, |K| = E
 *
 * Parameters
 * c_ik[N][E] - capabilities per agent
 * r_jk[M][E] - capabilities requirement per task
 * p_ij[N][M] - probability agent i can complete task j
 * d_j[M]     - Minimum number of agents required to complete task j
 */
MASPInput::MASPInput(std::string input_path) {
	// Initial assignment, silence annoying macro warnings
	// and avoid issues if parsing fails
	N = 0;
	M = 0;
	E = 0;
	c_ik = NULL;
	r_jk = NULL;
	p_ij = NULL;
	d_j = NULL;
	input_fileName = input_path;

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
		tpos_j - M x 2
		 ...
		apos_i - N x 2
		 ...

	  Example:
		# 6 agents, 2 tasks, 2 capabilities
		6 2 2
		# Agent capabilities
		1 0
		1 0
		0 1
		0 1
		1 1
		1 1
		# Task requirements
		1 0
		0 1
		# Probability agent i can complete task j
		0.95 0.0
		0.84 0.0
		0.0 0.90
		0.0 0.75
		0.85 0.91
		0.74 0.83
		# Minimum number of agents for each task
		2
		3
		# Location of each task: M x 2
		0 2000
		1000 -1000
		# Location of each drone: N x 2
		0 0
		1000 1000
		-1500 500
		-700 0
		-750 500
		-1000 -1000
	 */
	if(SANITY_PRINT)
		printf("Reading MASP input\n");

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
		printf(" N = %d, M = %d, E = %d\n", N, M, E);

	// Read in agent capabilities
	c_ik = new int*[N];
	// Sanity print
	if(DEBUG_MASPINPUT)
		printf(" c_ik:");
	for(int i = 0; i < N; i++) {
		c_ik[i] = new int[E];
		// Sanity print
		if(DEBUG_MASPINPUT)
			printf("\n  %d:\t", i);
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_c(line);
			// Expected E values for each c_ik[i]
			for(int k = 0; k < E; k++) {
				lineStream_c >> c_ik[i][k];
				// Sanity print
				if(DEBUG_MASPINPUT)
					printf("%d\t", c_ik[i][k]);
			}
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_MASPINPUT)
		puts("");

	// Read in role requirements
	r_jk = new int*[M];
	// Sanity print
	if(DEBUG_MASPINPUT)
		printf(" r_jk:");
	for(int j = 0; j < M; j++) {
		r_jk[j] = new int[E];
		// Sanity print
		if(DEBUG_MASPINPUT)
			printf("\n  %d:\t", j);
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_r(line);
			// Expected E values for each r_jk[i]
			for(int k = 0; k < E; k++) {
				lineStream_r >> r_jk[j][k];
				// Sanity print
				if(DEBUG_MASPINPUT)
					printf("%d\t", r_jk[j][k]);
			}
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_MASPINPUT)
		puts("");

	// Read in robot-to-role costs
	p_ij = new double*[N];
	// Sanity print
	if(DEBUG_MASPINPUT)
		printf(" p_ij:");
	for(int i = 0; i < N; i++) {
		p_ij[i] = new double[M];
		// Sanity print
		if(DEBUG_MASPINPUT)
			printf("\n  %d:\t", i);
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_d(line);
			// Expected N values for each d_ij[i]
			for(int j = 0; j < M; j++) {
				lineStream_d >> p_ij[i][j];
				// Sanity print
				if(DEBUG_MASPINPUT)
					printf("%.2f\t", p_ij[i][j]);
			}
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_MASPINPUT)
		puts("");

	// Read in robot-to-role costs
	d_j = new int[M];
	// Sanity print
	if(DEBUG_MASPINPUT)
		printf(" d_j:");
	for(int j = 0; j < M; j++) {
		// Sanity print
		if(DEBUG_MASPINPUT)
			printf("\n  %d:\t", j);
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_d(line);
			// Expect a single value for d_j
			lineStream_d >> d_j[j];
			// Sanity print
			if(DEBUG_MASPINPUT)
				printf("%d\t", d_j[j]);
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_MASPINPUT)
		puts("");

	// Read in task location
	tpos_j = new double*[M];
	// Sanity print
	if(DEBUG_MASPINPUT)
		printf(" tpos_j:");
	for(int j = 0; j < M; j++) {
		tpos_j[j] = new double[2];
		// Sanity print
		if(DEBUG_MASPINPUT)
			printf("\n  %d:\t", j);
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_d(line);
			// Expected 2 values
			double x, y;
			lineStream_d >> x;
			lineStream_d >> y;
			tpos_j[j][0] = x;
			tpos_j[j][1] = y;
			// Sanity print
			if(DEBUG_MASPINPUT)
				printf("%.2f\t%.2f", tpos_j[j][0], tpos_j[j][1]);
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_MASPINPUT)
		puts("");

	// Read in task location
	apos_i = new double*[N];
	// Sanity print
	if(DEBUG_MASPINPUT)
		printf(" apos_i:");
	for(int i = 0; i < N; i++) {
		apos_i[i] = new double[2];
		// Sanity print
		if(DEBUG_MASPINPUT)
			printf("\n  %d:\t", i);
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_d(line);
			// Expected 2 values
			double x, y;
			lineStream_d >> x;
			lineStream_d >> y;
			apos_i[i][0] = x;
			apos_i[i][1] = y;
			// Sanity print
			if(DEBUG_MASPINPUT)
				printf("%.2f\t%.2f", apos_i[i][0], apos_i[i][1]);
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}
	if(DEBUG_MASPINPUT)
		puts("");

	// Verify that we successfully read the input file
	if(!read_success) {
		// Input file not formatted correctly, hard fail!
		fprintf(stderr, "[MASPInput::MASPInput] : Input file format off\n");
		exit(1);
	}
	else {
		if(SANITY_PRINT)
			printf("Successfully read input\n\n");
	}
}

MASPInput::~MASPInput() {
	// Free memory
	delete[] d_j;

//	puts("p_ij:");
//	for(int i = 0; i < N; i++) {
//		printf(" @%p: ", (void *)(p_ij + i));
//		for(int j = 0; j < M; j++) {
//			printf("%.3f ", p_ij[i][j]);
//		}
//		puts("");
//		delete[] p_ij[i];
//	}
//	delete[] p_ij;

	for(int i = 0; i < N; i++) {
		delete[] p_ij[i];
	}
	delete[] p_ij;

	for(int j = 0; j < M; j++) {
		delete[] r_jk[j];
	}
	delete[] r_jk;
	for(int i = 0; i < N; i++) {
		delete[] c_ik[i];
	}
	delete[] c_ik;
}

// Probability that agent i can complete task j (input size: NxN)
double MASPInput::get_p_ij(int i, int j) {
	if(iCanDoj(i,j)) {
		return p_ij[i][j];
	}
	else {
		return 0;
	}
}

// Determines if i is capable of performing j
bool MASPInput::iCanDoj(int i, int j) {
	if(i < N && j < M) {
		// Iterate through capabilities
		for(int k = 0; k < E; k++) {
			// Check compatibility for capability k
			if(c_ik[i][k] - r_jk[j][k] < 0) {
				// Incompatible, end here
				return false;
			}
		}
		// If we made it this far, i can do j
		return true;
	}
	else {
		// i or j is out of bounds... You're doing something work -> hard fail!
		fprintf(stderr, "[MASPInput::iCanDoj] : Asked for bad i/j combo\n");
		exit(1);
	}
}

// Determines a theoretical upper bound on a possible solution
double MASPInput::UpperBound() {
	// Sanity print
	if(DEBUG_MASPINPUT)
		printf("Finding an upper bound on any solution\n");

	// Initially, the upper bound is 1
	double upperBound = 1;

	bool** x_ij = new bool*[N];

	// Sanity print
	if(DEBUG_MASPINPUT)
		printf("x_ij:\n");

	// Create an i to j mapping where any i that can do j is assigned to j
	for(int i = 0; i < N; i++) {
		x_ij[i] = new bool[M];
		// Check which tasks j can do
		for(int j = 0; j < M; j++) {
			// Can i do j?
			if(iCanDoj(i,j)) {
				x_ij[i][j] = true;
				// Sanity print
				if(DEBUG_MASPINPUT)
					printf(" 1");
			}
			else {
				// Sanity print
				if(DEBUG_MASPINPUT)
					printf(" 0");
				x_ij[i][j] = false;
			}
		}
		// Sanity print
		if(DEBUG_MASPINPUT)
			printf("\n");
	}

	// If any task has exactly d_j agents, free these agents from other assignments
	for(int j = 0; j < M; j++) {
		// Count have many agents are assigned to this task
		int agentsAssignedToTask = 0;
		std::vector<int> agentOnTask;
		for(int i = 0; i < N; i++) {
			if(x_ij[i][j]) {
				agentsAssignedToTask++;
				agentOnTask.push_back(i);
			}
		}
		// Is this the exact number required for the task?
		if(agentsAssignedToTask == d_j[j]) {
			// Sanity print
			if(DEBUG_MASPINPUT)
				printf(" Found a set of agents to free\n");

			// Free these agents from other tasks
			for(int i : agentOnTask) {
				for(int jj = 0; jj < M; jj++) {
					if(j != jj) {
						x_ij[i][jj] = false;
					}
				}
			}
		}
	}

	// Determine the probability that each task is complete
	PoissonBinomial poissonB;
	for(int j = 0; j < M; j++) {
		double prob_j = 0;
		// Get the number of agents/probabilities of completion for this task
		std::vector<double> p_values;
		int agentsAssignedToTask = 0;
		for(int i = 0; i < N; i++) {
			if(x_ij[i][j]) {
				agentsAssignedToTask++;
				p_values.push_back(p_ij[i][j]);
			}
		}
		for(int k = d_j[j]; k <= agentsAssignedToTask; k++) {
			//		PMF(int k, int n, std::vector<double>& p_values);
			prob_j += poissonB.PMF(k, agentsAssignedToTask, p_values);
		}
		upperBound *= prob_j;
	}

	// Memory clean-up
	for(int i = 0; i < N; i++) {
		delete[] x_ij[i];
	}
	delete[] x_ij;

	return upperBound;
}

