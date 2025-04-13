#include "HungGSSolver.h"

HungGSSolver::HungGSSolver(int nodeID, std::string input_path) : Node(nodeID), Solver(nodeID, input_path) {
	algorithm = e_Algo_HUNGGS;

	q_j = new double[N];
	M_i = -1;
	alpha_j = new double[N];
	beta_i = 1.1;
	minSlack_i = 1.1;
	n_i = -1;
	delta = 1.1;
	F2=true; // Start in F2
	F1_j = new bool[N];
	nFather = -1;
	nSon = -1;
	nOlderBrother = -1;
	nYungerBrother = -1;
	nFatherTask = -1;
	currentJ = -1;


	nOldFather = -1;
	nOldFatherTask = -1;
	nOldOlderBrother = -1;
	nOldYungerBrother = -1;
	nTempFather = -1;
	nTempSibling = -1;
}

HungGSSolver::~HungGSSolver() {
	delete[] alpha_j;
}

// Starts the gradient descent algorithm
void HungGSSolver::StartAlgo() {
	// Capture start time
	startTime = std::chrono::high_resolution_clock::now();

	/*
	 * Format of cost map (actually transposed)
	 *       1   2   3   4   5   6 :agents
	 * 1.1 q11 q21 q31 q41 q51 q61
	 * 1.2 q11 q21 q31 q41 q51 q61
	 * 2.1 q12 q22 q32 q42 q52 q62
	 * 2.2 ...
	 * 2.3
	 * x     1   1   1   1   1   1
	 * :tasks
	 *
	 * The costs are the probability that an agent fails to
	 * complete a task q_ij = (1-pij)
	 */
	// Create cost array (1xN where cost[j] is the prob. that agent i fails at task j)


	if(DEBUG_HUNGGS) {
		printf("[HUNGGS] Started Algorithm\n[HUNGGS]  Building cost map: ");
	}
	for(int j = 0; j < N; j++) {
		int taskJ = get_task(j);
		if(DEBUG_HUNGGS)
			printf("%d(%d): ", j, taskJ);
		if(taskJ >= 0) {
			if(iCanDoj(taskJ)) {
				// Regular task.. cost is probability of failure
				q_j[j] = 1 - get_p_j(taskJ);
			}
			else {
				// I can't do j... assign a weight of inf
				q_j[j] = INF;
			}
		}
		else {
			// Floating task.. push probability of 1
			q_j[j] = 1;
		}
		if(DEBUG_HUNGGS)
			printf("%f, ", q_j[j]);
	}


	/// Perform the initialization steps of the dist-Hungarian method
	// Let M_i := 0 (robot i is initially unmatched); let α_j := 0 for all the tasks j, and β_i := min_j{cij }.
	M_i = -1;
	for(int j = 0; j < N; j++) {
		alpha_j[j] = 0.0;
	}
	beta_i = 1.1;
	for(int j = 0; j < N; j++) {
		if(q_j[j] < beta_i) {
			beta_i = q_j[j];
		}
	}
	// Start in F2
	F2=true;
	for(int j = 0; j < N; j++) {
		// All trees start in
		F1_j[j] = true;
	}
	// Set robot i connected only with its younger (i − 1) and older brother robot (i + 1) and has no father and no sons.
	nFather = -1;
	nSon = -1;

	// Calculate [minSlack_i, n_i ] without exchanging any message with the others.
	minSlack_i = getMinSlack_i(&n_i);

	if(DEBUG_HUNGGS) {
		printf("\n[HUNGGS]  M_i = %d, alpha_j: ", M_i);
		for(int j = 0; j < N; j++) {
			printf("%f ", alpha_j[j]);
		}
		printf("\n[HUNGGS]  beta_i = %f, minSlack_i = %f, n_i =%d\n", beta_i, minSlack_i, n_i);
		printf("[HUNGGS]  q_j = ");
		for(int j = 0; j < N; j++) {
			printf("%f ", q_j[j]);
		}
		printf("\n");
	}

	// Are we node 0? The node that triggers everyone else...
	if(mNodeId == 0) {
		// Determine delta
		packet_t message;
		message.ID = 1; // First packet
		message.packetType = E_PacketType::e_HUNG_GETD_PCKT;
		// We need a packet_t point named "packet" for the macros...
		packet_t* packet = &message;
		// Put our minSlack_i as first value
		DELTA = minSlack_i;
		T_STAR = n_i;
		R_STAR = mNodeId;
		if(DEBUG_HUNGGS)
			printf("[HUNGGS]  Search for detla, currently: [%f (%d,%d)], packet-count = %d\n", DELTA, R_STAR, T_STAR, packet->ID);
		// Send the packet to our neighbor
		m_msgSender.SendMsg(NEIGHBOR, packet);
		// Always call return... just to be sure nothing goes weird..
		return;
	}
}

void HungGSSolver::RunAlgo(packet_t* packet) {
	if(packet->packetType == E_PacketType::e_HUNG_GETD_PCKT) {
		if(DEBUG_HUNGGS) {
			printf("[HUNGGS] New round, packet-count = %d\n", packet->ID);
			printf("[HUNGGS]   alpha_j = ");
			for(int j = 0; j < N; j++) {
				printf("%f ", alpha_j[j]);
			}
			printf("\n[HUNGGS]   beta_i = %f\n", beta_i);
			printf("[HUNGGS]   Tasks in F1 ? = ");
			for(int j = 0; j < N; j++) {
				printf("%d ", F1_j[j]);
			}
			printf("\n[HUNGGS]   Robot in F2 ? %d\n", F2);
			printf("[HUNGGS]   Family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);

			// Check for basic cycles
			if(nFather >= 0 && nFather == nSon) {
				fprintf(stderr, "[ERROR:HUNGGS] nFather -> nSon cycle!!\n");
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
			if(nFatherTask >= 0 && nFatherTask == M_i) {
				fprintf(stderr, "[ERROR:HUNGGS] nFatherTask == M_i!!\n");
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
			if(nOlderBrother >= 0 && nOlderBrother == nFather) {
				fprintf(stderr, "[ERROR:HUNGGS] nOlderBrother -> nFather cycle!!\n");
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
			if(nYungerBrother >= 0 && nYungerBrother == nFather) {
				fprintf(stderr, "[ERROR:HUNGGS] nYungerBrother -> nFather cycle!!\n");
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}

			printf("[HUNGGS]  Searching for delta, currently: [%f (%d,%d)]\n", DELTA, R_STAR, T_STAR);
		}

		// Reset random things..
		nOldFather = -1;
		nOldFatherTask = -1;
		nOldOlderBrother = -1;
		nOldYungerBrother = -1;
		nTempFather = -1;
		nTempSibling = -1;

		// We are searching for delta.. did this node start it?
		if(mNodeId == 0) {
			// We started the search... does delta exist? i.e. a task is not matched
			if(T_STAR >= 0) {
				// Record the result
				delta = DELTA;
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  * Found delta: [%f (%d,%d)], packet-count = %d\n", DELTA, R_STAR, T_STAR, packet->ID);
				// Report to the others that we found delta
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_REPD_PCKT;
				packet->misCounter = -1; // Report if they matched with our matched task
				packet->misCounter2 = -1; // Report if they matched with our parent task (and we are the oldest sibling
				packet->misCounter3 = -1; // This is to report a sibling (to pull things out of F1) -- Is this needed?
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
			else {
				// The algorithm ended!
				if(SANITY_PRINT) {
					printf("[HUNGGS]  No delta! [%f (%d,%d)], packet-count = %d\n", DELTA, R_STAR, T_STAR, packet->ID);
					printf("[HUNGGS] End Hungarian algorithm, collect solution!!!\n");
				}
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_CLCT_SOL_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
		}
		else {
			// Update minSlack
			minSlack_i = getMinSlack_i(&n_i);
			// Are we in F2?
			if(F2) {
				// Is our minSlack_i better?
				if(DELTA > minSlack_i) {
					// Update delta
					DELTA = minSlack_i;
					T_STAR = n_i;
					R_STAR = mNodeId;
					if(DEBUG_HUNGGS)
						printf("[HUNGGS]   We have a better delta [%f (%d,%d)], packet-count = %d\n", DELTA, R_STAR, T_STAR, packet->ID);
				}
			}
			// Pass this along to our neighbor
			packet->ID++;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_REPD_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Found delta: [%f (%d,%d)], packet-count = %d\n", DELTA, R_STAR, T_STAR, packet->ID);
		delta = DELTA;
		// Found delta, update alpha and maybe beta
		for(int j = 0; j < N; j++) {
			if(F1_j[j]) {
				alpha_j[j] = alpha_j[j] - delta;
			}
		}
		// Did we move into F1?
		if(!F2) {
			beta_i = beta_i-DELTA;
		}

		// If they matched with our task..
		if(T_STAR == M_i) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  * Report we are assigned to task %d\n", T_STAR);
			// Tell them they matched with our task
			packet->misCounter = mNodeId;
		}
		// If they matched with our father task
		else if(T_STAR == nFatherTask) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  * They matched with our parent task (%d)-[%d]\n", nFather, T_STAR);
			if(nOlderBrother == -1) {
				// We are the oldest sibling, report this to r*
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   We are the oldest sibling, inform r* of this\n");
				packet->misCounter2 = mNodeId;
			}
		}

		if(DEBUG_HUNGGS) {
			printf("[HUNGGS]  Updated alpha_j = ");
			for(int j = 0; j < N; j++) {
				printf("%f ", alpha_j[j]);
			}
			printf("\n[HUNGGS]   beta_i = %f\n", beta_i);
			printf("[HUNGGS]   Tasks in F1 ? = ");
			for(int j = 0; j < N; j++) {
				printf("%d ", F1_j[j]);
			}
			printf("\n[HUNGGS]   Robot in F2 ? %d\n", F2);
		}

		// Did this node start this...?
		if(mNodeId == 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Telling %d to start augmenting\n", R_STAR);
			// Tell r* to start augmenting the path
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_AUG_PCKT;
			m_msgSender.SendMsg(R_STAR, packet);
			return;
		}
		else {
			// Report to the others that we found delta
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_REPD_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_AUG_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Starting Augmenting with task %d, packet-count = %d\n", T_STAR, packet->ID);

		// Start finding augmented paths
		if(M_i == -1) {
			/// First time matching to a task! Start augmenting paths
			// Verify that we don't have any relationships
			if(nFather >= 0 || nOlderBrother >= 0 || nYungerBrother >= 0 || nSon >= 0) {
				// When you first match with a task you shouldn't have any family members
				fprintf(stderr, "[ERROR:HUNGGS] We have a family but we are unmatched...\n");
				fprintf(stderr, "[ERROR:HUNGGS]  Current family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}

			// Did we match with someone's matched task?
			if(packet->misCounter >= 0) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Tell node %d we want to augment\n", packet->misCounter);
				// Augment this connection
				nSon = packet->misCounter;
				// Match to t*
				M_i = T_STAR;
				// Record that t* will be moving into F2
				TASK_LIST[0] = T_STAR;
				packet->misCounter = 1;
				// Send the message to our new child that we want to augment tree
				packet->misCounter3 = -1; // No younger sibling
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_AUG_NXT_PCKT;
				m_msgSender.SendMsg(nSon, packet);
				return;
			}
			// Did we inherit a tree?
			else if(packet->misCounter2 >= 0) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Matched with %d, telling %d's tree to move to F2\n", T_STAR, packet->misCounter2);
				// We inherited a child
				nSon = packet->misCounter2;
				// Match to t*
				M_i = T_STAR;
				// Record that t* will be moving into F2
				TASK_LIST[0] = T_STAR;
				packet->misCounter = 1;

				// misCounter -> counter for buffer (has tasks to move to F2)
				// misCounter2 -> new parent (this node)
				// misCounter3 -> former younger sibling (none)
				packet->misCounter2 = mNodeId;
				packet->misCounter3 = -1;
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_SWP_F2_RT_PCKT;
				m_msgSender.SendMsg(nSon, packet);
				return;
			}
			else {
				// We are the first to match to this task
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Easy-matching with %d, tell everyone the task moved to F2\n", T_STAR);
				// Match to t*
				M_i = T_STAR;
				// Record that t* will be moving into F2
				TASK_LIST[0] = T_STAR;
				packet->misCounter = 1;
				// Tell everyone that T_STAR is in F2 now
				packet->ID++;
				packet->misCounter2 = mNodeId;
				packet->packetType = E_PacketType::e_HUNG_F2_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
		}
		// Already matched..
		else {
			if(DEBUG_HUNGGS) {
				printf("[HUNGGS]  Already matched to %d, moving to F1\n", M_i);
				printf("[HUNGGS]   Current family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
			}
			// Move to F1!
			F2 = false;
			nFatherTask = T_STAR;
			nTempFather = packet->misCounter;
			nTempSibling = packet->misCounter2;

			// If we are part of a larger tree... we need to disconnect from it
			if(nFather >= 0) {
				// Tell parent that we are leaving
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   Need to disconnect from parent (%d)\n", nFather);
				// Let them know who left
				packet->misCounter2 = mNodeId;
				// Let them know who our younger sibling was
				packet->misCounter3 = nYungerBrother;
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_LVE_FAM_PCKT;
				m_msgSender.SendMsg(nFather, packet);
				return;
			}
			// Else, if we are connecting to an established tree in F1, report to them
			else if(nTempFather >= 0 || nTempSibling >= 0) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   New parent (%d) and older sibling (%d) \n", nTempFather, nTempSibling);
				// Mark who our new parent is
				nFather = nTempFather;
				// We will join as the youngest (and maybe only) child
				nYungerBrother = -1;

				// Are we a younger sibling or are we the only child?
				if(nTempSibling >= 0) {
					// We have an older sibling, just join behind them
					if(DEBUG_HUNGGS)
						printf("[HUNGGS]    Join via sibling (%d) \n", nTempSibling);
					packet->misCounter2 = mNodeId; // Tells new tree who is joining
					packet->ID++;
					packet->packetType = E_PacketType::e_HUNG_NEW_SIB_F1_PCKT;
					m_msgSender.SendMsg(nTempSibling, packet);
					return;
				}
				else {
					// We are a new only-child
					if(DEBUG_HUNGGS)
						printf("[HUNGGS]    Join via parent (%d) \n", nTempFather);
					nOlderBrother = -1;
					// Report this to our new parent
					packet->ID++;
					packet->packetType = E_PacketType::e_HUNG_NEW_SON_F1_PCKT;
					m_msgSender.SendMsg(nFather, packet);
					return;
				}
			}
			// Else, if we have a sub-tree then bring them along
			else if(nSon >= 0) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   Tell child %d to move into F1\n", nSon);
				// Tell child to move into F1
				packet->misCounter = 2;
				TASK_LIST[0] = T_STAR;
				TASK_LIST[1] = M_i;
				// We are the root of this operation
				packet->misCounter2 = mNodeId;
				// Send a message to our parent that we moved into F1
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_MV_ST_F1_PCKT;
				m_msgSender.SendMsg(nSon, packet);
				return;
			}
			// Else... just move ourselves and our new parent-task into F1
			else {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   Tell everyone to move our tree of %d, %d into F1\n", T_STAR, M_i);
				// Set parent task
				nFatherTask = T_STAR;

				// Easy case, just tell everyone to move our child tasks to F1... decrease slack by delta
				packet->misCounter = 2;
				TASK_LIST[0] = T_STAR;
				TASK_LIST[1] = M_i;
				packet->ID++;
				packet->misCounter2 = mNodeId;
				packet->packetType = E_PacketType::e_HUNG_F1_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_DISCNT_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Disconnection message.. (sent from %d), packet-count = %d\n", packet->misCounter, packet->ID);
		// Are we the requester?
		if(mNodeId == packet->misCounter) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  We are the originators.\n");
			// We are the one who needs to disconnect
			if(nSon >= 0) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   Tell child %d we are disconnecting\n", nSon);
				// Tell child to move into F1
				packet->misCounter = 2;
				TASK_LIST[0] = T_STAR;
				TASK_LIST[1] = M_i;
				// Send a message to our parent that we moved into F1
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_MVDWN_F1_PCKT;
				m_msgSender.SendMsg(nSon, packet);
				return;
			}
			else {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   Tell everyone to move our tree of %d, %d into F1\n", T_STAR, M_i);
				// Easy case, just tell everyone to move our child tasks to F1... decrease slack by delta
				packet->misCounter = 2;
				TASK_LIST[0] = T_STAR;
				TASK_LIST[1] = M_i;
				packet->ID++;
				packet->misCounter2 = mNodeId;
				packet->packetType = E_PacketType::e_HUNG_F1_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
		}
		// Was it our child...?
		if(nSon == packet->misCounter) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Disconnecting from child %d\n", nSon);
			if(packet->misCounter2 >= 0) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   But got a new one! %d\n", packet->misCounter2);
				// New child
				nSon = packet->misCounter2;
				// Send packet to new first child
				packet->ID++;
				m_msgSender.SendMsg(nSon, packet);
				return;
			}
			else {
				// Very well, disconnect from child
				nSon = -1;
				// Send back acknowledgment
				packet->ID++;
				m_msgSender.SendMsg(packet->misCounter, packet);
				return;
			}
		}
		// Might have been our older sibling
		else if(nOlderBrother == packet->misCounter && packet->senderID == nFather) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Older sibling left %d\n", nOlderBrother);
			// Our older sibling left... so long!
			nOlderBrother = -1;
			// Respond that we disconnected them
			packet->ID++;
			m_msgSender.SendMsg(packet->misCounter, packet);
			return;
		}
		else {
			// Not an implemented case..
			fprintf(stderr, "[ERROR:HUNGGS] Asked to disconnect from someone weird.. %d\n", packet->misCounter);
			// Not implemented, initiate shut-down
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_LV_FAM_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Leaving family message, packet-count = %d\n", packet->ID);
		// Are we the initiator?
		if(mNodeId == packet->misCounter) {
			// We are the one who needs to disconnect
			if(nSon >= 0) {
				// Tell child to move into F1
				packet->misCounter = 2;
				TASK_LIST[0] = T_STAR;
				TASK_LIST[1] = M_i;
				// Send a message to our parent that we moved into F1
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_MVDWN_F1_PCKT;
				m_msgSender.SendMsg(nSon, packet);
				return;
			}
			else {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   Tell everyone to move our tree of %d, %d into F1\n", T_STAR, M_i);
				// Easy case, just tell everyone to move our child tasks to F1... decrease slack by delta
				packet->misCounter = 2;
				TASK_LIST[0] = T_STAR;
				TASK_LIST[1] = M_i;
				packet->ID++;
				packet->misCounter2 = mNodeId;
				packet->packetType = E_PacketType::e_HUNG_F1_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
		}
		// Was this our younger sibling?
		else if(nYungerBrother == packet->misCounter) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Younger sibling %d wants to leave\n", packet->misCounter);
			// Did they have a younger sibling?
			if(packet->misCounter2 >= 0) {
				// Yes, make them our younger sibling and tell them about it
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   New younger bro: %d, saying hello\n", packet->misCounter2);
				nYungerBrother = packet->misCounter2;
				packet->ID++;
				m_msgSender.SendMsg(packet->misCounter2, packet);
			}
			else {
				// No siblings, they just left
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   Acknowledging departure\n");
				nYungerBrother = -1;
				packet->ID++;
				m_msgSender.SendMsg(packet->misCounter, packet);
			}
		}
		// Was this our older sibling?
		else if(nOlderBrother == packet->misCounter) {
			// Then the packet sender must have been our new older brother
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Older brother %d left, have new older bro %d\n", packet->misCounter, packet->senderID);
			nOlderBrother = packet->senderID;
			// Acknowledge the first guy that we disconnected him
			packet->ID++;
			m_msgSender.SendMsg(packet->misCounter, packet);
		}
		else {
			// Not an implemented case..
			fprintf(stderr, "[ERROR:HUNGGS] Someone weird want to leave the family.. %d\n", packet->misCounter);
			fprintf(stderr, "[ERROR:HUNGGS] I don't know this person!\n");
			// Not implemented, initiate shut-down
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_AUG_UP_PCKT) {
		// Asked to augment our task
		if(DEBUG_HUNGGS) {
			printf("[HUNGGS] Asked to augment, packet-count = %d\n", packet->ID);
			printf("[HUNGGS]  Current Family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
				nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
		}

		// If we had siblings TODO: HERE!!

		if(nFather == -1 && nFatherTask >= 0 && nOlderBrother == -1 && nYungerBrother >= 0 && !F2) {
			// We are the oldest son without a family. Take our "father's" task, swap our younger brother to our son, move to F2
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Oldest son without a father, switch matching to %d\n", nFatherTask);
			// It is assumed that we don't have a child...
			if(nSon != packet->senderID) {
				// Found ourselves in a weird scenario...
				fprintf(stderr, "[ERROR:HUNGGS] Augmenting Up and expected younger bro (%d) to become son, but child (%d) != (%d) sender!\n", nYungerBrother, nSon, packet->senderID);
				// Not implemented, initiate shut-down
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
			F2 = true;
			int oldMatch = M_i;
			M_i = nFatherTask;
			// Update who our father is (should be Node we just received packet from)
			nFather = packet->senderID;
			nFatherTask = oldMatch;
			nSon = nYungerBrother;
			nYungerBrother = -1;
			// Add our new match into queue
			TASK_LIST[packet->misCounter] = M_i;
			packet->misCounter++;
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]   Current Family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
			// Tell brother they are now our son, and should move into F2
			packet->misCounter2 = nFather; // Report new parent
			packet->packetType = E_PacketType::e_HUNG_MVLAT_F2_PCKT;
			packet->ID++;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		else if(nFather == -1 && nFatherTask >= 0 && !F2) {
			// No father but have father task -> switch our matching and move to F2
			if(DEBUG_HUNGGS) {
				printf("[HUNGGS]  No father, switch matching to %d\n", nFatherTask);
			}
			F2 = true;
			int oldMatch = M_i;
			M_i = nFatherTask;
			// Update who our father is (should be Node we just received packet from)
			nFather = packet->senderID;
			nFatherTask = oldMatch;
			// Add our new match into queue
			TASK_LIST[packet->misCounter] = M_i;
			packet->misCounter++;
			// Was this our child?
			if(nSon == packet->senderID) {
				// Tell everyone to update which task are in F2
				packet->misCounter2 = mNodeId;
				packet->packetType = E_PacketType::e_HUNG_F2_PCKT;
				packet->ID++;
				m_msgSender.SendMsg(NEIGHBOR, packet);
			}
			else if(nSon >= 0) {
				if(nYungerBrother >= 0) {
					// Not sure what to do here..
					fprintf(stderr, "[ERROR:HUNGGS] Need to assign son as sibling %d, but already have sibling %d\n", nSon, nYungerBrother);
					// Not implemented, initiate shut-down
					packet->ID++;
					packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
					m_msgSender.SendMsg(NEIGHBOR, packet);
					return;
				}
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]   Tell child we are siblings %d\n", nSon);
				// Tell child they are now our sibling.. and move into F2
				nYungerBrother = nSon;
				packet->misCounter2 = nFather; // Report new parent
				packet->packetType = E_PacketType::e_HUNG_MVDWN_F2_PCKT;
				packet->ID++;
				m_msgSender.SendMsg(nSon, packet);
			}
			// No more child
			nSon = -1;
			return;
		}
		else if(nFather >= 0 && nFatherTask >= 0 && !F2) {
			// Switch our matching, move to F2, tell parent
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Switch matching to %d, tell parent %d\n", nFatherTask, nFather);
			F2 = true;
			int oldMatch = M_i;
			M_i = nFatherTask;
			// Update who our father is (should be Node we just received packet from)
			nSon = nFather;
			nFather = packet->senderID;
			nFatherTask = oldMatch;
			// Add our new match into queue
			TASK_LIST[packet->misCounter] = M_i;
			packet->misCounter++;
			// Tell everyone to update which task are in F2
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_AUG_UP_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		else {
			// Unexpected scenario...
			fprintf(stderr, "[ERROR:HUNGGS] Got augment request but something is wrong..\n");
			fprintf(stderr, "[ERROR:HUNGGS]  father = %d, father-task = %d, F2? %d\n", nFather, nFatherTask, F2);
			// Not implemented, initiate shut-down
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_F2_PCKT) {
		if(DEBUG_HUNGGS) {
			printf("[HUNGGS] Moving tasks to F2\n");
			printf("[HUNGGS]  Current family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n", nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
			printf("[HUNGGS]  Moving tasks: ");
		}
		// Update which tasks are now in F2
		for(int l = 0; l < packet->misCounter; l++) {
			int j = TASK_LIST[l];
			F1_j[j] = false;
			if(DEBUG_HUNGGS)
				printf("%d ", j);
		}
		if(DEBUG_HUNGGS)
			printf(", packet-count = %d\n", packet->ID);

		// Did we start initiate this (win the round)?
		if(packet->misCounter2 == mNodeId) {
			// We did. This means the round is over
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Round is over, telling 0 to initiate a new round\n");
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_NEW_RND_PCKT;
			m_msgSender.SendMsg(0, packet);
			return;
		}
		else {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Telling neighbor (%d)\n", NEIGHBOR);
			packet->ID++;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_NEW_RND_PCKT) {
		// Start a new round
		if(mNodeId == 0) {
			if(DEBUG_HUNGGS) {
				// Ask everyone to validate their relationships
				printf("[HUNGGS] Tell everyone to validate their relationships, packet-count = %d\n", packet->ID);
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_CHECK_FAM_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
			else {
				// Update minSlack
				minSlack_i = getMinSlack_i(&n_i);
				// Reset delta
				DELTA = 10000;
				R_STAR = -1;
				T_STAR = -1;
				// Are we in F2?
				if(F2) {
					// Set our minSlack as the current delta
					DELTA = minSlack_i;
					R_STAR = mNodeId;
					T_STAR = n_i;
				}
				// Send the packet to our neighbor
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_GETD_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
		}
		else {
			// Not Node-0.. Why was I asked to start a new round???
			fprintf(stderr, "[ERROR:HUNGGS] Node (%d) asked to start new round, not node 0!\n", mNodeId);
			// Not expected, initiate shut-down
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_CHECK_FAM_PCKT) {
		// Check each relationship in this order: Father, older brother, younger brother, son
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Verifying relationships, packet-count = %d\n", packet->ID);
		packet_t familyPacket;
		familyPacket.packetType = E_PacketType::e_HUNG_RELTNSHP_CHECK_PCKT;

		if(nFather >= 0 && nOlderBrother == -1) {
			// Send message to parent to validate that they consider us their son
			familyPacket.misCounter = 0;
			// Validate what task they have
			familyPacket.misCounter2 = nFatherTask;
			m_msgSender.SendMsg(nFather, &familyPacket);
		}
		if(nOlderBrother >= 0) {
			// Send message to older brother to validate that they consider us their younger brother
			familyPacket.misCounter = 1;
			m_msgSender.SendMsg(nOlderBrother, &familyPacket);
		}
		if(nYungerBrother >= 0) {
			// Send message to older brother to validate that they consider us their younger brother
			familyPacket.misCounter = 2;
			m_msgSender.SendMsg(nYungerBrother, &familyPacket);
		}
		if(nSon >= 0) {
			// Send message to son to validate that they consider us their father
			familyPacket.misCounter = 3;
			// Validate that they are tracking our matched task
			familyPacket.misCounter2 = M_i;
			m_msgSender.SendMsg(nSon, &familyPacket);
		}

		// Did we start this mess?
		if(mNodeId == 0) {
			// Actually start a new round now
			if(DEBUG_HUNGGS)
				printf("[HUNGGS] Starting a new round now\n");

			// Update minSlack
			minSlack_i = getMinSlack_i(&n_i);
			// Reset delta
			DELTA = 10000;
			R_STAR = -1;
			T_STAR = -1;
			// Are we in F2?
			if(F2) {
				// Set our minSlack as the current delta
				DELTA = minSlack_i;
				R_STAR = mNodeId;
				T_STAR = n_i;
			}
			if(DEBUG_HUNGGS) {
				printf("[HUNGGS]  Search for delta, currently: [%f (%d,%d)], packet-count = %d\n", DELTA, R_STAR, T_STAR, packet->ID);
			}
			// Send the packet to our neighbor
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_GETD_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
		// No.. pass message onto next person
		else {
			packet->ID++;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_RELTNSHP_CHECK_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Asked to validate relationship with (%d), packet-count = %d\n", packet->senderID, packet->ID);
		switch(packet->misCounter) {
		// Parent check (they should be our son)
		case 0:
			if(nSon == packet->senderID && packet->misCounter2 == M_i) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  This node is our child -- good relationship!\n");
			}
			else {
				fprintf(stderr, "[ERROR:HUNGGS] Node [%d]->(%d) is not our child!\n", packet->misCounter2, packet->senderID);
				fprintf(stderr, "[ERROR:HUNGGS]  Current family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
			}
			return;
		// Older brother check (they should be our younger brother)
		case 1:
			if(nYungerBrother == packet->senderID) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  This node is our younger brother -- good relationship!\n");
			}
			else {
				fprintf(stderr, "[ERROR:HUNGGS] Node (%d) is not our younger brother!\n", packet->senderID);
				fprintf(stderr, "[ERROR:HUNGGS]  Current family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
			}
			return;
		// Younger brother check (they should be our older brother)
		case 2:
			if(nOlderBrother == packet->senderID) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  This node is our older brother -- good relationship!\n");
			}
			else {
				fprintf(stderr, "[ERROR:HUNGGS] Node (%d) is not our older brother!\n", packet->senderID);
				fprintf(stderr, "[ERROR:HUNGGS]  Current family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
			}
			return;
		// Child check (they should be our father)
		case 3:
			if(nFather == packet->senderID && packet->misCounter2 == nFatherTask) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  This node is our father -- good relationship!\n");
			}
			else {
				fprintf(stderr, "[ERROR:HUNGGS] Node (%d)->[%d] is not our parent!\n", packet->senderID, packet->misCounter2);
				fprintf(stderr, "[ERROR:HUNGGS]  Current family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
				packet->ID++;
				packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
			}
			return;
		default:
			// Something went wrong here...
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_F1_PCKT) {
		// Mark the tasks in packet as F1
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Moving back to F1: ");
		// Update which tasks are now in F2
		for(int l = 0; l < packet->misCounter; l++) {
			int j = TASK_LIST[l];
			if(!F1_j[j]) {
				F1_j[j] = true;
				// Reduce their alpha
				alpha_j[j] = alpha_j[j] - delta;
			}
			if(DEBUG_HUNGGS)
				printf("%d ", j);
		}
		if(DEBUG_HUNGGS)
			printf(", packet-count = %d\n", packet->ID);

		// Did we start this?
		if(packet->misCounter2 == mNodeId) {
			// We did, start a new round
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell 0 to start new round\n");
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_NEW_RND_PCKT;
			m_msgSender.SendMsg(0, packet);
			return;
		}
		else {
			// Nope.. tell our neighbor
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Telling neighbor\n");
			packet->ID++;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_MVDWN_F1_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Asked by parent to move into F1...\n");
		if(!F2) {
			// Should have been in F2...
			fprintf(stderr, "[ERROR:HUNGGS] Asked to move into F1 but we are already here...\n");
			fprintf(stderr, "[ERROR:HUNGGS]  M_i = %d\n", M_i);
			// Not expected, initiate shut-down
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}

		// Move into F1
		F2 = false;

		// Report our matched task
		TASK_LIST[packet->misCounter] = M_i;
		packet->misCounter++;

		// Do we have siblings?
		if(nOlderBrother >= 0 || nYungerBrother >= 0) {
			// TODO: handle siblings
			fprintf(stderr, "[ERROR:HUNGGS] Asked to move into F1 and we have a sibling: %d,%d...\n", nOlderBrother, nYungerBrother);
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
		if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Pass message onto parent %d\n", nFather);
			// Send a message to our child that we moved into F1
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MVDWN_F1_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		else {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  End of tree, tell the others\n");
			// No children! Tell everyone what to move into F1
			packet->ID++;
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_F1_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_MVDWN_F2_PCKT) {
		// Asked to move out of F1!
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Asked to move into F2, packet-count = %d\n", packet->ID);
		// Move ourself into F2
		F2 = true;
		// Record our matched node
		TASK_LIST[packet->misCounter] = M_i;
		packet->misCounter++;
		// Do we have a sibling (I hope not..)
		if(nOlderBrother >= 0 || nYungerBrother >= 0) {
			// TODO: Need to handle siblings...
			fprintf(stderr, "[ERROR:HUNGGS] Asked to move into F2 and we have a sibling: %d...\n", nYungerBrother);
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
		// Do we have a new parent?
		if(packet->misCounter2 >= 0) {
			// Move former parent to older sibling..
			nOlderBrother = nFather;
			// New parent
			nFather = packet->misCounter2;
		}
		// Do we have a child?
		if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Telling child %d to move into F2\n", nSon);
			packet->ID++;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		else {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  No children, move to F2\n");
			// No children, tell everyone what moved into F2
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_F2_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_OLDR_SIB_PCKT) {
		// Looks like what we thought was our father is actually our older brother...
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Swapping father (%d) for older brother, packet-count = %d\n", nFather, packet->ID);
		if(nOlderBrother >= 0) {
			// We already have an older sibling... this shouldn't be the case!
			fprintf(stderr, "[ERROR:HUNGGS] We already have an older sibling!! (%d) <-- (%d<-%d->%d)\n",
					nFather, nOlderBrother, mNodeId, nYungerBrother);
			// Not implemented, initiate shut-down
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}

		// Correct relationships
		nOlderBrother = nFather;
		nFather = -1;

		// Do we need to inform our siblings?
		if(nYungerBrother >= 0) {
			// Tell younger sibling
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_CRRCT_FTHR_PCKT;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
		else {
			// Tell oldest brother to finish augmenting
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MV_SUBT_PCKT;
			m_msgSender.SendMsg(nOlderBrother, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_CRRCT_FTHR_PCKT) {
		// Looks like what we thought was our father is actually an older brother...
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Lost our father.. packet-count = %d\n", packet->ID);
		// Correct relationships
		int oldestBrother = nFather;
		nFather = -1;

		// Do we need to inform the next sibling?
		if(nYungerBrother >= 0) {
			// Tell younger brother
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_CRRCT_FTHR_PCKT;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
		else {
			// Tell oldest brother to finish augmenting
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MV_SUBT_PCKT;
			m_msgSender.SendMsg(oldestBrother, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_MV_SUBT_PCKT) {
		// We need to move ourselves and our sub-tree into F1
		printf("[HUNGGS] Moving sub-tree into F1, packet-count = %d\n", packet->ID);
		if(nFather >= 0) {
			// We need to disconnect from our parent
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell father (%d) we are disconnecting from F2!\n", nFather);
			// Send a message to our parent that we are disconnecting and moving into F1
			packet->misCounter = mNodeId;
			// They don't have a new child (this case would have triggered a shut-down earlier)
			packet->misCounter2 = -1;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_DISCNT_PCKT;
			m_msgSender.SendMsg(nFather, packet);
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]   Remove old father\n");
			nFather = -1;

			return;
		}
		else if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell child %d to move into F1\n", nSon);
			// Tell child to move into F1
			packet->misCounter = 1;
			TASK_LIST[0] = M_i;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MVDWN_F1_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		else {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell everyone to move this robot and our task (%d) into F1\n", M_i);
			// Easy case, just tell everyone to move our child tasks to F1... decrease slack by delta
			packet->misCounter = 1;
			TASK_LIST[0] = M_i;
			packet->ID++;
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_F1_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_MVLAT_F2_PCKT) {
		// Our older sibling has become our father...
		printf("[HUNGGS] Brother (%d) is now our father, move sub-tree into F1, packet-count = %d\n", nOlderBrother, packet->ID);

		// Check weird edge cases that we haven't implemented...
		if(nYungerBrother >= 0) {
			// TODO: Handle larger spanning tree...
			fprintf(stderr, "[ERROR:HUNGGS] We don't handle larger spanning tree! Family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
					nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
			// Not implemented, initiate shut-down
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}

		// Move ourself into F2
		F2 = true;
		// Set older brother as parent
		nFather = nOlderBrother;
		nOlderBrother = -1;
		// Record our matched node
		TASK_LIST[packet->misCounter] = M_i;
		packet->misCounter++;

		if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Telling child %d to move to F2\n", nSon);
			// Tell children to move into F2
			packet->misCounter2 = -1;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MVDWN_F2_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;

			return;
		}
		else {
			// End of the line.. report who is all moving with us
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell everyone to move this robot and our task (%d) into F2\n", M_i);
			// Easy case, just tell everyone to move our child tasks to F1... decrease slack by delta
			packet->misCounter2 = mNodeId;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_F2_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_AUG_NXT_PCKT) {
		// We are augmenting and moving out of F1!
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Asked to augment by %d, packet-count = %d\n", packet->senderID, packet->ID);
		// Record old parent and parent's task
		nOldFather = nFather;
		nOldFatherTask = nFatherTask;
		nOldOlderBrother = nOlderBrother;
		nOldYungerBrother = nYungerBrother;
		// Fix family, sender becomes parent, matched task is now parents task
		nFather = packet->senderID;
		nFatherTask = M_i;
		// Swap tasks
		M_i = nOldFatherTask;
		// No older brother
		nOlderBrother = -1;
		if(nSon == packet->senderID) {
			// Son's younger brother becomes your younger brother
			nYungerBrother = packet->misCounter3;
		}
		else {
			// Son becomes younger brother
			nYungerBrother = nSon;
		}
		// Former parent is now our son
		nSon = nOldFather;
		TASK_LIST[packet->misCounter] = M_i;
		packet->misCounter++;
		// Update to F2
		F2 = true;
		if(nYungerBrother >= 0) {
			// Report to siblings that things changed
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell next sibling (%d) to move into F2, update family\n", nYungerBrother);
			// misCounter -> counter for buffer (has tasks to move to F2)
			// misCounter2 -> new parent
			// misCounter3 -> The younger sibling of the new parent
			packet->misCounter2 = nFather;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_UD_FAM_F2_PCKT;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
		else {
			/// Augment up
			if(nSon >= 0) {
				// Tell next node to augment
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Augment up to (%d) \n", nSon);
				packet->misCounter3 = nOldYungerBrother;
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_AUG_NXT_PCKT;
				m_msgSender.SendMsg(nSon, packet);
				return;
			}
			else {
				// We were father-less... We need to tell our former siblings about the swap
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  In weird father-less case...\n");
				if(nOldOlderBrother >= 0) {
					// Need to move to left-most sibling
					if(DEBUG_HUNGGS)
						printf("[HUNGGS]  Moving to left-most sibling, starting with (%d)\n", nOldOlderBrother);
					// misCounter -> counter for buffer (has tasks to move to F2)
					// misCounter2 -> new parent (this node)
					// misCounter3 -> former younger sibling
					packet->misCounter2 = mNodeId;
					packet->misCounter3 = nOldYungerBrother;
					packet->ID++;
					packet->packetType = E_PacketType::e_HUNG_SWP_F2_LF_PCKT;
					m_msgSender.SendMsg(nOldOlderBrother, packet);
					return;
				}
				else if(nOldYungerBrother >= 0) {
					// Tell former younger sibling (now our son) to move
					if(DEBUG_HUNGGS)
						printf("[HUNGGS]  Telling younger sibling (%d) -- they are our child now\n", nOldYungerBrother);
					nSon = nOldYungerBrother;
					// misCounter -> counter for buffer (has tasks to move to F2)
					// misCounter2 -> new parent (this node)
					// misCounter3 -> former younger sibling
					packet->misCounter2 = mNodeId;
					packet->misCounter3 = nOldYungerBrother;
					packet->ID++;
					packet->packetType = E_PacketType::e_HUNG_SWP_F2_RT_PCKT;
					m_msgSender.SendMsg(nSon, packet);
					return;
				}
				else {
					// Done, report to everyone what moved into F2
					if(DEBUG_HUNGGS)
						printf("[HUNGGS]  Done augmenting, report what moved\n");
					packet->misCounter2 = mNodeId;
					packet->packetType = E_PacketType::e_HUNG_F2_PCKT;
					m_msgSender.SendMsg(NEIGHBOR, packet);
					return;
				}
			}
		}
	}
	// Tells former children of a node that the parent augmented, switched to be a sibling, and we have a new parent
	else if(packet->packetType == E_PacketType::e_HUNG_UD_FAM_F2_PCKT) {
		// We are moving out of F1!
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Asked to update family by %d, moving into F2, packet-count = %d\n", packet->senderID, packet->ID);
		// Update to F2
		F2 = true;
		// Add our match onto queue
		TASK_LIST[packet->misCounter] = M_i;
		packet->misCounter++;
		// Update parent
		nOldFather = nFather;
		nFather = packet->misCounter2;
		// Update sibling -- the sender is now your older brother
		nOlderBrother = packet->senderID;
		// Do we have a child?
		if(nSon >= 0) {
			// Tell sub-tree
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell child (%d) to move into F2, update family\n", nSon);
			// misCounter -> counter for buffer
			// misCounter2 -> this node ID (the root of the sub-tree)
			packet->misCounter2 = mNodeId;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MV_SUBT_F2_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		// Else.. either tell next sibling or report back to old parent
		else {
			contMovingF2(packet);
			return;
		}
	}
	// Used to recursively move a sub-tree into F2
	else if(packet->packetType == E_PacketType::e_HUNG_MV_SUBT_F2_PCKT) {
		// Are we the root?
		if(mNodeId == packet->misCounter2) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS] Successfully moved sub-tree into F2, packet-count = %d\n", packet->ID);
			contMovingF2(packet);
			return;
		}
		// Did we already move into F2?
		if(F2) {
			// We must have already told our children to move.. tell sibling.. or parent
			if(DEBUG_HUNGGS)
				printf("[HUNGGS] Told to move to F2 but we already did, packet-count = %d\n", packet->ID);
			if(nYungerBrother >= 0) {
				// Send message to next sibling
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Passing message along to younger brother (%d)\n", nYungerBrother);
				packet->ID++;
				m_msgSender.SendMsg(nYungerBrother, packet);
				return;
			}
			else {
				// Pass message back up to parent
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Passing message back to parent (%d)\n", nFather);
				packet->ID++;
				m_msgSender.SendMsg(nFather, packet);
				return;
			}
		}

		// Move into F2
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Moving into F2, packet-count = %d\n", packet->ID);
		F2 = true;
		// Add our match onto queue
		TASK_LIST[packet->misCounter] = M_i;
		packet->misCounter++;
		// Tell child to move.. or next sibling.. or go back to parent
		if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Passing message to child (%d)\n", nSon);
			packet->ID++;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		else if(nYungerBrother >= 0) {
			// Send message to next sibling
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Passing message to younger brother (%d)\n", nYungerBrother);
			packet->ID++;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
		else {
			// Pass message back up to parent
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Passing message back to parent (%d)\n", nFather);
			packet->ID++;
			m_msgSender.SendMsg(nFather, packet);
			return;
		}
	}
	// Tells node to finish augmenting
	else if(packet->packetType == E_PacketType::e_HUNG_CNTU_AUG_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Done augmenting, packet-count = %d\n", packet->ID);
		// If we had a parent (now our child)...
		if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell former parent (%d) to augment next\n", nSon);
			// Tell them to augment
			packet->misCounter3 = nOldYungerBrother; // Record former younger sibling
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_AUG_NXT_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		// Were we father-less?
		else if(nOldFather == -1 && nOldOlderBrother >= 0) {
			// Need to move to left-most sibling
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Moving to left-most sibling, starting with (%d)\n", nOldOlderBrother);
			// misCounter -> counter for buffer (has tasks to move to F2)
			// misCounter2 -> new parent (this node)
			// misCounter3 -> former younger sibling
			packet->misCounter2 = mNodeId;
			packet->misCounter3 = nOldYungerBrother;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_SWP_F2_LF_PCKT;
			m_msgSender.SendMsg(nOldOlderBrother, packet);
			return;
		}
		else if(nOldFather == -1 && nOldYungerBrother >= 0) {
			// Tell former younger sibling (now our son) to move
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Telling younger sibling (%d) -- they are our child now\n", nOldYungerBrother);
			nSon = nOldYungerBrother;
			// misCounter -> counter for buffer (has tasks to move to F2)
			// misCounter2 -> new parent (this node)
			// misCounter3 -> former younger sibling
			packet->misCounter2 = mNodeId;
			packet->misCounter3 = nOldYungerBrother;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_SWP_F2_RT_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		// Nothing else to update
		else {
			// Done, report to everyone what moved into F2
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tree ended, report what moved\n");
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_F2_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	// Move to left-most node
	else if(packet->packetType == E_PacketType::e_HUNG_SWP_F2_LF_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Finding left-most parent-less child, packet-count = %d\n", packet->ID);
		if(nOlderBrother >= 0) {
			// Pass on to next sibling
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Passing message to next sibling (%d)\n", nOlderBrother);
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_SWP_F2_LF_PCKT;
			m_msgSender.SendMsg(nOlderBrother, packet);
			return;
		}
		else {
			// We are the oldest sibling.. make change and move back right
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  We are left-most, reporting back to parent (%d)\n", packet->misCounter2);
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_LFT_REP_FTHR_PCKT;
			m_msgSender.SendMsg(packet->misCounter2, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_HUNG_LFT_REP_FTHR_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Found new child (%d), packet-count = %d\n", packet->senderID, packet->ID);
		nSon = packet->senderID;
		// Tell former younger sibling (now our son) to move
		if(DEBUG_HUNGGS)
			printf("[HUNGGS]  Telling child (%d) that they are our child\n", nSon);
		// misCounter -> counter for buffer (has tasks to move to F2)
		// misCounter2 -> new parent (this node)
		// misCounter3 -> former younger sibling
		packet->misCounter2 = mNodeId;
		packet->misCounter3 = nOldYungerBrother;
		packet->ID++;
		packet->packetType = E_PacketType::e_HUNG_SWP_F2_RT_PCKT;
		m_msgSender.SendMsg(nSon, packet);
		return;
	}
	// Move to left-most packet
	else if(packet->packetType == E_PacketType::e_HUNG_SWP_F2_RT_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Parent-less child, move to F2, packet-count = %d\n[HUNGGS]  Packet: sndr = %d, cntr2 = %d, cntr3 = %d\n", packet->ID, packet->senderID, packet->misCounter2, packet->misCounter3);
		/// Perform parent-less augment
		// Switch to F2
		F2 = true;
		// Add our match onto queue
		TASK_LIST[packet->misCounter] = M_i;
		packet->misCounter++;
		// Update parent
		nFather = packet->misCounter2;
		// Are we the first to get this message?
		if(packet->misCounter2 == packet->senderID) {
			// We are now the oldest
			nOlderBrother = -1;
		}
		else {
			// This message came from our new older brother
			nOlderBrother = packet->senderID;
		}
		// Was our parent formally our younger brother?
		if(nYungerBrother == nFather) {
			// Update update younger brother
			nYungerBrother = packet->misCounter3;
		}

		if(DEBUG_HUNGGS)
			printf("[HUNGGS]  New parent = %d, Siblings: (%d <- -> %d)\n", nFather, nOlderBrother, nYungerBrother);

		if(nSon >= 0) {
			// Tell sub-tree
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell sub-tree with son (%d) to move into F2\n", nSon);
			// misCounter -> counter for buffer
			// misCounter2 -> this node ID (the root of the sub-tree)
			packet->misCounter2 = mNodeId;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MV_LST_SUBT_F2_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		// Do we have a younger sibling?
		if(nYungerBrother >= 0) {
			// Pass to next node
			// misCounter -> counter for buffer (has tasks to move to F2)
			// misCounter2 -> our parent
			// misCounter3 -> former younger sibling (already set)
			packet->misCounter2 = nFather;
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_SWP_F2_RT_PCKT;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
		else {
			// Finished augmenting... report all tasks that moved into F2
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_F2_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	// This packet tells the new parent who the oldest child is
	else if(packet->packetType == E_PacketType::e_HUNG_REPRT_SON_PCKT) {
		// Tell former younger sibling (now our son) to more
		nSon = packet->senderID;
		// misCounter -> counter for buffer (has tasks to move to F2)
		// misCounter2 -> new parent (this node)
		// misCounter3 -> former younger sibling
		packet->misCounter2 = mNodeId;
		packet->misCounter3 = nOldYungerBrother;
		packet->ID++;
		packet->packetType = E_PacketType::e_HUNG_SWP_F2_RT_PCKT;
		m_msgSender.SendMsg(nOldOlderBrother, packet);
		return;
	}
	// Used to move the last sub-trees over into F2
	else if(packet->packetType == E_PacketType::e_HUNG_MV_LST_SUBT_F2_PCKT) {
		// Are we the root?
		if(mNodeId == packet->misCounter2) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS] Successfully moved last sub-tree into F2, packet-count = %d\n", packet->ID);
			// Do we have a younger sibling?
			if(nYungerBrother >= 0) {
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Telling next node (%d) to go\n", nYungerBrother);
				// Pass to next node
				// misCounter -> counter for buffer (has tasks to move to F2)
				// misCounter2 -> our parent
				// misCounter3 -> former younger sibling (already set)
				packet->misCounter2 = nFather;
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_SWP_F2_RT_PCKT;
				m_msgSender.SendMsg(nYungerBrother, packet);
				return;
			}
			else {
				// Finished augmenting... report all tasks that moved into F2
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Done augmenting! Report F2\n");
				packet->misCounter2 = mNodeId;
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_F2_PCKT;
				m_msgSender.SendMsg(NEIGHBOR, packet);
				return;
			}
		}
		// Did we already move into F2?
		if(F2) {
			// We must have already told our children to move.. tell sibling.. or parent
			if(DEBUG_HUNGGS)
				printf("[HUNGGS] (last tree) Told to move to F2 but we already did, packet-count = %d\n", packet->ID);
			if(nYungerBrother >= 0) {
				// Send message to next sibling
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Passing message along to younger brother (%d)\n", nYungerBrother);
				packet->ID++;
				m_msgSender.SendMsg(nYungerBrother, packet);
				return;
			}
			else {
				// Pass message back up to parent
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Passing message back to parent (%d)\n", nFather);
				packet->ID++;
				m_msgSender.SendMsg(nFather, packet);
				return;
			}
		}

		// Move into F2
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] (last tree) Moving into F2, packet-count = %d\n", packet->ID);
		F2 = true;
		// Add our match onto queue
		TASK_LIST[packet->misCounter] = M_i;
		packet->misCounter++;
		// Tell child to move.. or next sibling.. or go back to parent
		if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Passing message to child (%d)\n", nSon);
			packet->ID++;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		else if(nYungerBrother >= 0) {
			// Send message to next sibling
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Passing message to younger brother (%d)\n", nYungerBrother);
			packet->ID++;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
		else {
			// Pass message back up to parent
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Passing message back to parent (%d)\n", nFather);
			packet->ID++;
			m_msgSender.SendMsg(nFather, packet);
			return;
		}
	}
	// This packet tells a parent that one of their children has left
	else if(packet->packetType == E_PacketType::e_HUNG_LVE_FAM_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Child (%d) asked to leave, packet-count = %d\n", packet->senderID, packet->ID);
		// If this was our first child, then update children
		if(packet->senderID == nSon) {
			// Update first child (reported from child leaving)
			nSon = packet->misCounter3;
		}
		// Send message on to next child (which we may not actually have)
		if(nSon >= 0) {
			// Inform child that someone left
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell oldest child (%d) about the change\n", nSon);
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_KCK_BRTHR_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		else {
			// No more children, tell that original guy to finish moving into F1
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  No children, tell (%d) to keep moving!\n", packet->misCounter2);
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_KCKD_FRM_FAM_PCKT;
			m_msgSender.SendMsg(packet->misCounter2, packet);
			return;
		}
	}
	// This packet tells a child that one of their siblings left
	else if(packet->packetType == E_PacketType::e_HUNG_KCK_BRTHR_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Sibling (%d) asked to leave, packet-count = %d\n", packet->misCounter2, packet->ID);
		/// Correct family
		// Are we the first child to know?
		if(packet->senderID == nFather) {
			// Then we are the oldest
			nOlderBrother = -1;
		}
		else {
			// Set sender as older brother
			nOlderBrother = packet->senderID;
		}
		// Is the exiting node our former younger brother?
		if(nYungerBrother == packet->misCounter2) {
			nYungerBrother = packet->misCounter3;
		}

		// Send message on to next sibling
		if(nYungerBrother >= 0) {
			// Inform child that someone left
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell next sibling (%d) about the change\n", nYungerBrother);
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_KCK_BRTHR_PCKT;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
		else {
			// No more siblings, tell that original guy to finish moving into F1
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Telling (%d) to keep moving\n", packet->misCounter2);
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_KCKD_FRM_FAM_PCKT;
			m_msgSender.SendMsg(packet->misCounter2, packet);
			return;
		}
	}
	// This packet tells the exiting node to continue exiting
	else if(packet->packetType == E_PacketType::e_HUNG_KCKD_FRM_FAM_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] We have successfully left our old family, packet-count = %d\n", packet->ID);
		// We shouldn't have a parent or children at this point..
		nFather = -1;
		nOlderBrother = -1;
		nYungerBrother = -1;

		// Are we connecting to an established tree in F1?
		if(nTempFather >= 0 || nTempSibling >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]   New parent (%d) and older sibling (%d) \n", nTempFather, nTempSibling);
			// Mark who our new parent is
			nFather = nTempFather;
			// We will join as the youngest (and maybe only) child
			nYungerBrother = -1;

			// Are we a younger sibling or are we the only child?
			if(nTempSibling >= 0) {
				// We have an older sibling, just join behind them
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]    Join via sibling (%d) \n", nTempSibling);
				packet->misCounter2 = mNodeId; // Tells new tree who is joining
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_NEW_SIB_F1_PCKT;
				m_msgSender.SendMsg(nTempSibling, packet);
				return;
			}
			else {
				// We are a new only-child
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]    Join via parent (%d) \n", nTempFather);
				nOlderBrother = -1;
				// Report this to our new parent
				packet->ID++;
				packet->packetType = E_PacketType::e_HUNG_NEW_SON_F1_PCKT;
				m_msgSender.SendMsg(nFather, packet);
				return;
			}
		}
		// Else, if we have a sub-tree then bring them along
		else if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]   Tell child %d to move into F1\n", nSon);
			// Tell child to move into F1
			packet->misCounter = 2;
			TASK_LIST[0] = T_STAR;
			TASK_LIST[1] = M_i;
			// We are the root of this operation
			packet->misCounter2 = mNodeId;
			// Send a message to our parent that we moved into F1
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MV_ST_F1_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		// Else... just move ourselves and our new parent-task into F1
		else {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]   Tell everyone to move our tree of %d, %d into F1\n", T_STAR, M_i);
			// Set parent task
			nFatherTask = T_STAR;

			// Easy case, just tell everyone to move our child tasks to F1... decrease slack by delta
			packet->misCounter = 2;
			TASK_LIST[0] = T_STAR;
			TASK_LIST[1] = M_i;
			packet->ID++;
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_F1_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	// This packet is used to connect a node to a new sibling in F1
	else if(packet->packetType == E_PacketType::e_HUNG_NEW_SIB_F1_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Node (%d) wants to join family, packet-count = %d\n", packet->misCounter2, packet->ID);
		// Do we already have a younger brother?
		if(nYungerBrother >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Tell younger sibling (%d)\n", nYungerBrother);
			// Pass the message along
			packet->ID++;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
		else {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  We have a new younger sibling (%d)\n", packet->misCounter2);
			// Nope, make this new guy our younger sibling
			nYungerBrother = packet->misCounter2;
			// Tell this new node to finish moving
			packet->packetType = E_PacketType::e_HUNG_FNSH_MV_F1_PCKT;
			packet->ID++;
			m_msgSender.SendMsg(nYungerBrother, packet);
			return;
		}
	}
	// Tells node that they have a new child in F1
	else if(packet->packetType == E_PacketType::e_HUNG_NEW_SON_F1_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] New child (%d) in F1, packet-count = %d\n", packet->senderID, packet->ID);
		if(nSon >= 0) {
			// This isn't correct...
			fprintf(stderr, "[ERROR:HUNGGS] We already have a child!!\n");
			fprintf(stderr, "[ERROR:HUNGGS]  Current family: (%d)-[%d]->(%d <-%d-> %d)-[%d]->(%d)\n",
				nFather, nFatherTask, nOlderBrother, mNodeId, nYungerBrother, M_i, nSon);
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
		}

		// Mark the sender as our child
		nSon = packet->senderID;
		// Tell them to finish moving over
		packet->packetType = E_PacketType::e_HUNG_FNSH_MV_F1_PCKT;
		packet->ID++;
		m_msgSender.SendMsg(nSon, packet);
		return;
	}
	// Tells node that they have joined a new family in F1
	else if(packet->packetType == E_PacketType::e_HUNG_FNSH_MV_F1_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] We have successfully joined a new family, packet-count = %d\n", packet->ID);
		// Did this come from our parent or a sibling?
		if(packet->senderID != nFather) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  Marking (%d) are our new older brother\n", packet->senderID);
			nOlderBrother = packet->senderID;
		}
		// Do we have a sub-tree then bring along?
		if(nSon >= 0) {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]   Tell child %d to move into F1\n", nSon);
			// Tell child to move into F1
			packet->misCounter = 2;
			TASK_LIST[0] = T_STAR;
			TASK_LIST[1] = M_i;
			// We are the root of this operation
			packet->misCounter2 = mNodeId;
			// Send a message to our parent that we moved into F1
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_MV_ST_F1_PCKT;
			m_msgSender.SendMsg(nSon, packet);
			return;
		}
		// Else... just move ourselves and our new parent-task into F1
		else {
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]   Tell everyone to move our tree of %d, %d into F1\n", T_STAR, M_i);
			// Set parent task
			nFatherTask = T_STAR;

			// Easy case, just tell everyone to move our child tasks to F1... decrease slack by delta
			packet->misCounter = 2;
			TASK_LIST[0] = T_STAR;
			TASK_LIST[1] = M_i;
			packet->ID++;
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_F1_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	// Used to move a sub-tree into F1
	else if(packet->packetType == E_PacketType::e_HUNG_MV_ST_F1_PCKT) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS] Told to move sub-tree into F1, packet-count = %d\n", packet->ID);
		// Are we the root node?
		if(mNodeId == packet->misCounter2) {
			// If we get back to this case, then we have reached the end of the tree search
			if(DEBUG_HUNGGS)
				printf("[HUNGGS]  We started this process... report what moved into F1\n");
			// Inform everyone about what all moved into F1
			packet->misCounter2 = mNodeId;
			packet->packetType = E_PacketType::e_HUNG_F1_PCKT;
			packet->ID++;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
		// Else, did we already get this message?
		else if(!F2) {
			// Send to sibling (or back to parent)
			if(nYungerBrother >= 0) {
				// Send to next sibling
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Already moved, pass message onto younger brother (%d)\n", nYungerBrother);
				packet->ID++;
				m_msgSender.SendMsg(nYungerBrother, packet);
				return;
			}
			else {
				// Send back to parent
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Already moved, pass message back to parent (%d)\n", nFather);
				packet->ID++;
				m_msgSender.SendMsg(nFather, packet);
				return;
			}
		}
		else {
			// Move to F1
			F2 = false;
			TASK_LIST[packet->misCounter] = M_i;
			packet->misCounter++;

			// Finish moving to F1, then send to child or sibling.. or back to parent
			if(nSon >= 0) {
				// Tell child to move
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Moved, tell child (%d) to move to F1\n", nSon);
				packet->ID++;
				m_msgSender.SendMsg(nSon, packet);
				return;
			}
			else if(nYungerBrother >= 0) {
				// Send to next sibling
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Moved, tell younger brother (%d) to move to F1\n", nYungerBrother);
				packet->ID++;
				m_msgSender.SendMsg(nYungerBrother, packet);
				return;
			}
			else {
				// Send back to parent
				if(DEBUG_HUNGGS)
					printf("[HUNGGS]  Moved, send back to parent (%d)\n", nFather);
				packet->ID++;
				m_msgSender.SendMsg(nFather, packet);
				return;
			}
		}
	}




	///
	// Distributed Hungarian Method end case!
	///
	else if(packet->packetType == E_PacketType::e_HUNG_CLCT_SOL_PCKT) {
		// Algorithm finished, add our data to the table
		int j = get_task(M_i);
		X_I(mNodeId).task = j;
		currentJ = j;
		// Did we get paired with a real task?
		if(j >= 0) {
			// Put probability of success into table
			X_I(mNodeId).probability = get_p_j(X_I(mNodeId).task);
		}
		else {
			// No real task.. put in 0
			X_I(mNodeId).probability = 0.0;
		}
		if(SANITY_PRINT)
			printf("[HUNGGS] Collecting Results, paired with task %d (%d)\n", M_i, j);

		// Did we start this message?
		if(this->mNodeId == 0) {
			// We started these messages
			if(SANITY_PRINT) {
				printf("Hungarian Method Result:\n");
				for(int i = 0; i < N; i++) {
					printf(" %d: %d - %f\n", i, X_I(i).task, X_I(i).probability);
				}
			}

			// Start the GS
			packet->ID++;
			packet->misCounter = 0;
			packet->packetType = E_PacketType::e_GSR_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
		else {
			// Pass message along to next person
			packet->ID++;
			packet->packetType = E_PacketType::e_HUNG_CLCT_SOL_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
			return;
		}
	}
	else if(packet->packetType == E_PacketType::e_GSR_PCKT) {
		/// Run Gradient Search algorithm
		bool madeChange = true;

		// Debug print
		if(DEBUG_HUNGGS) {
			printf("[GS] Running gradient assent\n");
		}

		// Erase this agent's entries
		for(int j = 0; j < M; j++) {
			X_I(mNodeId).task = -1;
			X_I(mNodeId).probability = 0.0;
		}

		// Debug print
		if(DEBUG_HUNGGS) {
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
				if(DEBUG_HUNGGS)
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
					if(DEBUG_HUNGGS) {
						printf("[GS]  * joining task %d\n", j);
					}

					// Did we actually change anything...?
					if(j == currentJ) {
						// We did not...
						madeChange = false;

						// Debug print
						if(DEBUG_HUNGGS) {
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
			if(DEBUG_HUNGGS) {
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
				if(DEBUG_HUNGGS) {
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
				if(DEBUG_HUNGGS) {
					printf("[GS]   * Found best Z = %f at %d\n", bestZ, bestJ);
				}

				// Did we actually change anything...?
				if(bestJ == currentJ) {
					// We did not...
					madeChange = false;

					// Debug print
					if(DEBUG_HUNGGS) {
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
			packet->misCounter = packet->ID + N; // Record the number of packets to complete the algorithm
			packet->ID++;
			packet->packetType = E_PacketType::e_STOP_ALGO_PCKT;
			m_msgSender.SendMsg(NEIGHBOR, packet);
		}
		else {
			// Keep running the algorithm
			if(DEBUG_HUNGGS) {
				printf("[GS]   Round %d, Z = %f, rounds w/o change = %d\n", packet->ID+N, currentZ, packet->misCounter);
			}

			// Pass the message forward
			packet->ID++;
			m_msgSender.SendMsg(NEIGHBOR, packet);
		}

	}
	else if(packet->packetType == E_PacketType::e_STOP_ALGO_PCKT) {
		// We found a solution, stop the algorithm
		// Tell neighbor to shut down
		packet->ID++;
		m_msgSender.SendMsg(NEIGHBOR, packet);
		// Start shut-down
		runNode = false;

		// Try to record solution (may fail here...)
		if(mNodeId == 0) {
			// Capture end time
			auto stop = std::chrono::high_resolution_clock::now();
			// Determine the time it took to solve this
			long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime).count();
			double duration_s = (double)duration/1000.0;

			// Record how long you have been on..
			if(PRINT_RESULTS) {
				// Check status
				double Z = BenchmarkCF((agent_tp*)packet->buffer);

				FILE * pOutputFile;
				if(SANITY_PRINT) {
					printf("[HUNGGS] Told to stop algorithm, printing results\n");
					printf("[HUNGGS]  Found solution: %.10f %d %.10f\n", Z, packet->misCounter, duration_s);
				}
				pOutputFile = fopen("alg_2.dat", "a");
				// File format: n m computed-Z packets-sent run-time
				fprintf(pOutputFile, "%d %d %.10f %d %.10f %s\n", N, M, Z, packet->misCounter, duration_s, sInput.c_str());
				fclose(pOutputFile);
			}

			if(SANITY_PRINT)
				printf("[HUNGGS] Shut-down initiated after %.10f s\n", duration_s);
		}
		else {
			if(SANITY_PRINT)
				printf("[HUNGGS] Told to shut-down\n");
		}

		return;
	}
	else {
		// Not sure what this was...
		if(SANITY_PRINT)
			printf("[HUNGGS] Received unexpected message type: %d\n", packet->packetType);
		return;
	}
}

// minSlack_i = min_j∈F_1 (c_ij + α_j − β_i ) and n_i be the task corresponding to the arg min
double HungGSSolver::getMinSlack_i(int* n_i) {
	double minVal = 1000000;
	*n_i = -1;
	for(int j = 0; j < N; j++) {
		// Is this task in F1?
		if(F1_j[j]) {
			double val_j = q_j[j] + alpha_j[j] - beta_i;
			if(val_j < minVal) {
				minVal = val_j;
				*n_i = j;
			}
		}
	}

	return minVal;
}

/*
 * Takes cost matrix index and converts it into the corresponding task's index. This assumes
 * that the entries in the matrix are in order based on minimum required agents per task
 * (e.g. 1.1 1.2 2.1 2.2 .. x.1 x.2 ..). It will return -1 if the given index is past the
 * minimum number of agents required over tasks (in x range from example).
 */
int HungGSSolver::get_task(int index) {
	// Verify that we were given a valid input
	if((index < 0) || (index >= N)) {
		// Something went wrong -> hard fail!
		fprintf(stderr, "[MASP_MatchAct::get_task] : Bad index = %d\n", index);
		exit(1);
	}

	// Run through each task
	int run_j = 0;
	for(int j = 0; j < M; j++) {
		for(int jj = 0; jj < d_j[j]; jj++) {
			if(index == run_j) {
				return j;
			}
			run_j++;
		}
	}

	return -1;
}

// We turned the problem into a balanced matching problem, so there may not actually be a task j
double HungGSSolver::getMtch_q_j(int j) {
	double retVal = 1;



	return retVal;
}


/*
 * After a node is notified to move out of F2, it needs to move the rest of its sub-tree out
 * of F2 then notify either its next sibling or the former parent. This function performs
 * the last part of this process.
 */
void HungGSSolver::contMovingF2(packet_t* packet) {
	// If new parent is our former younger sibling..
	if(nFather == nYungerBrother) {
		// Then update younger sibling
		nYungerBrother = packet->misCounter3;
	}

	// Pass message along or tell former parent to continue augmenting
	if(nYungerBrother >= 0) {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS]  Passing message to next sibling (%d)\n", nYungerBrother);
		// Note who the new parent is
		packet->misCounter2 = nFather;
		packet->ID++;
		packet->packetType = E_PacketType::e_HUNG_UD_FAM_F2_PCKT;
		m_msgSender.SendMsg(nYungerBrother, packet);
	}
	else {
		if(DEBUG_HUNGGS)
			printf("[HUNGGS]  Telling old parent (%d) to continue augmenting\n", nOldFather);
		packet->ID++;
		packet->packetType = E_PacketType::e_HUNG_CNTU_AUG_PCKT;
		m_msgSender.SendMsg(nOldFather, packet);
	}
}
