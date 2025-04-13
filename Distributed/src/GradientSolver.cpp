#include "GradientSolver.h"

GradientSolver::GradientSolver(int nodeID, std::string input_path) : Node(nodeID), Solver(nodeID, input_path) {
	currentJ = -1;
	algorithm = e_Algo_GS;
}

GradientSolver::~GradientSolver() {}

// Starts the gradient descent algorithm
void GradientSolver::StartAlgo() {
	// Capture start time
	startTime = std::chrono::high_resolution_clock::now();

	// Are we node 0? The node that triggers everyone else...
	if(mNodeId == 0) {
		// Create the assignment table
		packet_t message;
		message.ID = 1; // First packet
		message.packetType = E_PacketType::e_GSR_PCKT;
		// We need a packet_t point named "packet" for the macros...
		packet_t* packet = &message;
		// Reset all table entries
		for(int i = 0; i < N; i++) {
			X_I(i).task = -1;
			X_I(i).probability = 0.0;
		}

		// Search for our favorite task
		int bestJ = -1;
		double bestP_ij = 0.0;
		for(int j = 0; j < M; j++) {
			if(iCanDoj(j)) {
				if(get_p_j(j) > bestP_ij) {
					bestP_ij = get_p_j(j);
					bestJ = j;
				}
			}
		}

		// Did we find a task?
		if(bestJ >= 0) {
			// Assign ourselves to our favorite task
			X_I(mNodeId).task = bestJ;
			X_I(mNodeId).probability = bestP_ij;
			currentJ = bestJ;

			if(DEBUG_GS)
				printf("[GS] Assigned self to task %d, p_ij = %f\n", bestJ, bestP_ij);
		}
		else {
			// This agent cannot contribute... Just pass along the matrix and hope for the best
			if(DEBUG_GS)
				printf("[GS] This node cannot contribute to the problem...\n");
		}

		// Send matrix to next agent to continue algorithm
		m_msgSender.SendMsg(NEIGHBOR, packet);
	}
}

void GradientSolver::RunAlgo(packet_t* packet) {
	if(packet->packetType == E_PacketType::e_GSR_PCKT) {
		// Continue with algorithm
	}
	else if(packet->packetType == E_PacketType::e_STOP_ALGO_PCKT) {
		// We found a solution, stop the algorithm
		if(mNodeId == 0) {
			// Capture end time
			auto stop = std::chrono::high_resolution_clock::now();
			// Determine the time it took to solve this
			long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime).count();
			double duration_s = (double)duration/1000.0;

			// Record solution
			double currentZ = BenchmarkCF((agent_tp*)packet->buffer);

			// Print results to file
			if(PRINT_RESULTS) {
				FILE * pOutputFile;
				if(SANITY_PRINT)
					printf("[GS]  Printing results\n");
				pOutputFile = fopen("alg_1.dat", "a");
				// File format: n m computed-Z packets-sent run-time
				// Still need to tell all nodes the algorithm ended... so we will be sending another N packets
				fprintf(pOutputFile, "%d %d %.10f %d %.10f %s\n", N, M, currentZ, packet->ID, duration_s, sInput.c_str());
				fclose(pOutputFile);
			}

			if(SANITY_PRINT)
				printf("[GS] Shut-down initiated after %.10f s\n", duration_s);
		}

		// Tell neighbor to shut down
		// NOTE: Someone else already updated the packet count
		m_msgSender.SendMsg(NEIGHBOR, packet);
		// Start shut-down
		runNode = false;

		return;
	}
	else {
		// Not sure what this was...
		if(SANITY_PRINT)
			printf("[GS] Received unexpected message type: %d\n", packet->packetType);
		return;
	}

	/// Run Gradient Search algorithm
	bool madeChange = true;

	// Debug print
	if(DEBUG_GS) {
		printf("[GS] Running gradient assent\n");
	}

	// Erase this agent's entries
	for(int j = 0; j < M; j++) {
		X_I(mNodeId).task = -1;
		X_I(mNodeId).probability = 0.0;
	}

	// Debug print
	if(DEBUG_GS) {
		printf("[GS]  previously assigned to %d\n", currentJ);
	}

	// Attempt to join any suitable task that has < d_j agents
	bool joinedTask = false;
	{
		// Check our favorite task, if it already has d_j agents then search next fav task o.w assign to fav
		std::priority_queue<std::tuple<float, int>> pQ;
		for(int j = 0; j < M; j++) {
			if(iCanDoj(j)) {
				std::tuple<double, int> tup(get_p_j(j), j);
				pQ.push(tup);
			}
		}

		// Try to join a task
		while(!pQ.empty() && !joinedTask) {
			// Get our next favorite task
			auto n = pQ.top();
			int j = std::get<1>(n);
			pQ.pop();

			// Sanity print
			if(DEBUG_GS)
				printf("[GS]  I want task %d, with %f\n", std::get<1>(n), std::get<0>(n));

			// How many agents are already assigned to j?
			int assigedToJ = 0;
			for(int i = 0; i < N; i++) {
				if(X_I(i).task == j){
					assigedToJ++;
				}
			}

			// If there is still room for this agent...
			if(assigedToJ < d_j[j]) {
				// Assign this agent to j
				X_I(mNodeId).task = j;
				X_I(mNodeId).probability = get_p_j(j);
				joinedTask = true;

				// Debug print
				if(DEBUG_GS) {
					printf("[GS]  * joining task %d\n", j);
				}

				// Did we actually change anything...?
				if(j == currentJ) {
					// We did not...
					madeChange = false;

					// Debug print
					if(DEBUG_GS) {
						printf("[GS]    No change..\n");
					}
				}

				// Update which task we joined
				currentJ = j;
			}
			// ELSE: Try the next task...
		}
	}

	// If no such task exists... Cycle through each to see if we can make an improvement
	if(!joinedTask) {
		// Debug print
		if(DEBUG_GS) {
			printf("[GS]  All easy entries are full...\n");
		}

		if(currentJ == -1) {
			// We weren't assigned yet, just join our favorite task for now...
			double bestP = -1;
			int bestJ = -1;
			for(int j = 0; j < M; j++) {
				if(iCanDoj(j)) {
					if(get_p_j(j) > bestP) {
						bestP = get_p_j(j);
						bestJ = j;
					}
				}
			}

			// Assign i to its favorite task
			X_I(mNodeId).task = bestJ;
			X_I(mNodeId).probability = bestP;
			currentJ = bestJ;

			// Debug print
			if(DEBUG_GS) {
				printf("[GS]   * just join favorite task %d\n", bestJ);
			}
		}
		else {
			double bestZ = -1;
			int bestJ = -1;
			// For each task...
			for(int j = 0; j < M; j++) {
				if(iCanDoj(j)) {
					// Try to assign this agent to the task
					X_I(mNodeId).task = j;
					X_I(mNodeId).probability = get_p_j(j);
					//X_IJ(mNodeId, j) = get_p_j(j);
					// Check if this improved the performance
					double Z = BenchmarkCF((agent_tp*)packet->buffer);
					if(Z > bestZ) {
						// Found a better spot
						bestZ = Z;
						bestJ = j;
					}
					// Reset
					X_I(mNodeId).task = -1;
					X_I(mNodeId).probability = 0.0;
					//X_IJ(mNodeId, j) = 0.0;
				}
			}
			// Assign ourselves to the task where we had the greatest impact
			X_I(mNodeId).task = bestJ;
			X_I(mNodeId).probability = get_p_j(bestJ);
			//X_IJ(mNodeId, bestJ) = get_p_j(bestJ);

			// Debug print
			if(DEBUG_GS) {
				printf("[GS]   * Found best Z = %f at %d\n", bestZ, bestJ);
			}

			// Did we actually change anything...?
			if(bestJ == currentJ) {
				// We did not...
				madeChange = false;

				// Debug print
				if(DEBUG_GS) {
					printf("[GS]    No change..\n");
				}
			}

			// Update our current task
			currentJ = bestJ;
		}
	}

	if(madeChange) {
		packet->misCounter = 0;
	}
	else {
		packet->misCounter++;
	}

	// Check status
	double currentZ = BenchmarkCF((agent_tp*)packet->buffer);

	// Check if we reached a stopping point...
	if(packet->misCounter > N) {
		if(SANITY_PRINT) {
			printf("[GS] Algorithm Complete\n");
			printf("[GS]  Z = %f, total packets = %d\n", currentZ, packet->ID+N);
		}

		// We made it the entire way around without making a change. Terminate the algorithm
		packet->ID += N; // We will send another n more packets... just mark it now, the others will stop counting
		packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
		m_msgSender.SendMsg(NEIGHBOR, packet);
	}
	else {
		// Keep running the algorithm
		if(SANITY_PRINT) {
			printf("[GS]   Round %d, Z = %f, rounds w/o change = %d\n", packet->ID+N, currentZ, packet->misCounter);
		}

		// Pass the message forward
		packet->ID++;
		m_msgSender.SendMsg(NEIGHBOR, packet);
	}
}

