/*
 * defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			January 12, 2024
 *
 * Description: Global project defines.
 */

#pragma once

#include <cstring>


#define DEBUG			0
#define SANITY_PRINT	0

#define EPSILON			0.000001
#define INF				1000000000000
#define PI				3.14159265

#define MAX_MSG_SIZE 2800

// struct for routing table entry
struct table_entry_t {
	char sIP[16];
	int nPort;
};

// Default routing table
#define NUM_AGENTS		48
table_entry_t const peer_table[] = {{"127.0.0.1", 80810}, {"127.0.0.1", 80811}, {"127.0.0.1", 80812}, {"127.0.0.1", 80813}, {"127.0.0.1", 80814}, {"127.0.0.1", 80815}, {"127.0.0.1", 80816}, {"127.0.0.1", 80817},{"127.0.0.1", 80820}, {"127.0.0.1", 80821}, {"127.0.0.1", 80822}, {"127.0.0.1", 80823}, {"127.0.0.1", 80824}, {"127.0.0.1", 80825}, {"127.0.0.1", 80826}, {"127.0.0.1", 80827},{"127.0.0.1", 80830}, {"127.0.0.1", 80831}, {"127.0.0.1", 80832}, {"127.0.0.1", 80833}, {"127.0.0.1", 80834}, {"127.0.0.1", 80835}, {"127.0.0.1", 80836}, {"127.0.0.1", 80837},{"127.0.0.1", 80840}, {"127.0.0.1", 80841}, {"127.0.0.1", 80842}, {"127.0.0.1", 80843}, {"127.0.0.1", 80844}, {"127.0.0.1", 80845}, {"127.0.0.1", 80846}, {"127.0.0.1", 80847},{"127.0.0.1", 80850}, {"127.0.0.1", 80851}, {"127.0.0.1", 80852}, {"127.0.0.1", 80853}, {"127.0.0.1", 80854}, {"127.0.0.1", 80855}, {"127.0.0.1", 80856}, {"127.0.0.1", 80857},{"127.0.0.1", 80860}, {"127.0.0.1", 80861}, {"127.0.0.1", 80862}, {"127.0.0.1", 80863}, {"127.0.0.1", 80864}, {"127.0.0.1", 80865}, {"127.0.0.1", 80866}, {"127.0.0.1", 80867}};

//table_entry_t const peer_table[] = {{"127.0.0.1", 8080}, {"127.0.0.1", 8081}, {"127.0.0.1", 8082}, {"127.0.0.1", 8083}, {"127.0.0.1", 8084}, {"127.0.0.1", 8085}, {"127.0.0.1", 8086}, {"127.0.0.1", 8087}};

// List of implemented algorithms
enum {
	e_Algo_P2P = 0,
	e_Algo_GS = 1,
	e_Algo_HUNGGS = 2
};

// List of packet types
enum E_PacketType {
	// Maintenance type messages
	e_NULL_PCKT = 0,		// NULL packet
	e_SHUT_DOWN_PCKT,	// Shuts down the node
	e_MAX_MAINT_PCKT,
	// Algorithm type messages
	e_MESSAGING_PCKT,	// Indicates to run message passing example
	e_STOP_ALGO_PCKT,	// Stops the assignment algorithm (triggers shutdown)
	e_GSR_PCKT,		// Gradient Descent search algorithm
	e_HUNG_GETD_PCKT,	// Dist. Hungarian Method: search for delta
	e_HUNG_REPD_PCKT,	// Dist. Hungarian Method: report delta found
	e_HUNG_AUG_PCKT, 	// Dist. Hungarian Method: start augmenting paths
	e_HUNG_F2_PCKT, 	// Dist. Hungarian Method: report tasks moving into F2
	e_HUNG_NEW_RND_PCKT, // Dist. Hungarian Method: start a new round
	e_HUNG_CHECK_FAM_PCKT, // Dist. Hungarian Method: Tells this node to validate its relatives (used in debug mode)
	e_HUNG_RELTNSHP_CHECK_PCKT, // Dist. Hungarian Method: Used to check a relationship (type stored in misCounter)
	e_HUNG_F1_PCKT,	 // Dist. Hungarian Method: tell everyone that we moved to F1...
	e_HUNG_MVDWN_F1_PCKT, // Dist. Hungarian Method: tell parent to move to F1...
	e_HUNG_AUG_UP_PCKT, // Dist. Hungarian Method: Augment tree
	e_HUNG_MVDWN_F2_PCKT, // Dist. Hungarian Method: Tell children to move into F2
	e_HUNG_DISCNT_PCKT, // Dist. Hungarian Method: Tell parent we are disconnecting
	e_HUNG_LV_FAM_PCKT, // Dist. Hungarian Method: Tell siblings we are leaving
	e_HUNG_OLDR_SIB_PCKT, // Dist. Hungarian Method: Already matched and inherited a sibling (who thinks we are their father)
	e_HUNG_CRRCT_FTHR_PCKT, // Dist. Hungarian Method: Tell siblings they don't have a father
	e_HUNG_MV_SUBT_PCKT, // Dist. Hungarian Method: Tell the oldest sibling (who was in F2) to move its sub-tree to F1
	e_HUNG_MVLAT_F2_PCKT, // Dist. Hungarian Method: Tell younger sibling to move its sub-tree to F2
	e_HUNG_AUG_NXT_PCKT, // Dist. Hungarian Method: Tell other node to augment
	e_HUNG_UD_FAM_F2_PCKT, // Dist. Hungarian Method: Tell sibling to update family, move into F2
	e_HUNG_MV_SUBT_F2_PCKT, // Dist. Hungarian Method: DFS through a sub-tree to update to F2
	e_HUNG_CNTU_AUG_PCKT, // Dist. Hungarian Method: Tells former parent (now older sibling) to continue augmenting
	e_HUNG_SWP_F2_LF_PCKT, // Dist. Hungarian Method: Move to left-most sibling to swap parent (swap moving right)
	e_HUNG_LFT_REP_FTHR_PCKT, // Dist. Hungarian Method: When we reach the left-most node, they need to report back to the parent
	e_HUNG_SWP_F2_RT_PCKT, // Dist. Hungarian Method: Move right, swapping parent (and sibling)
	e_HUNG_SWP_F2_RT_FST_PCKT, // Dist. Hungarian Method: Packet from new parent to move right, swapping parent (and sibling)
	e_HUNG_REPRT_SON_PCKT, // Dist. Hungarian Method: Tells former sibling (now parent) that we are their oldest son
	e_HUNG_MV_LST_SUBT_F2_PCKT, // Dist. Hungarian Method: Tells former sibling (now parent) that we are their oldest son
	e_HUNG_LVE_FAM_PCKT, // Dist. Hungarian Method: Tells parent that the sender is leaving
	e_HUNG_KCK_BRTHR_PCKT, // Dist. Hungarian Method: One of our siblings left for F1...
	e_HUNG_KCKD_FRM_FAM_PCKT, // Dist. Hungarian Method: Tells a node that is leave for F1 that they have been kicked out of the family
	e_HUNG_NEW_SIB_F1_PCKT, // Dist. Hungarian Method: Tells a node that they are getting a new younger brother (in F1)
	e_HUNG_NEW_SON_F1_PCKT, // Dist. Hungarian Method: Tells new parent that they have a child in F1
	e_HUNG_FNSH_MV_F1_PCKT, // Dist. Hungarian Method: Tells the node that joined a tree in F1 to finish moving
	e_HUNG_MV_ST_F1_PCKT, // Dist. Hungarian Method: Tells sub-tree to move into F1
	// Used to start the GSR algorithm:
	e_HUNG_CLCT_SOL_PCKT, // Dist. Hungarian Method: The algorithm has ended, collect the solution!
};



// Data packet struct
struct packet_t {
	// Packet ID
	int ID;
	int senderID;
	// Packet type
	E_PacketType packetType;
	// A miscellaneous counters
	int misCounter;
	int misCounter2;
	int misCounter3;
	// Packet data buffer
	char buffer[MAX_MSG_SIZE];

	// Generic constructor
	packet_t() {
		ID = 0;
		senderID = 0;
		misCounter = 0;
		misCounter2 = 0;
		misCounter3 = 0;
		packetType = E_PacketType::e_NULL_PCKT;
		memset(buffer, 0, MAX_MSG_SIZE);
	}
	// Copy constructor
	packet_t(const packet_t& other) {
		ID = other.ID;
		senderID = other.senderID;
		misCounter = other.misCounter;
		misCounter2 = other.misCounter2;
		misCounter3 = other.misCounter3;
		packetType = other.packetType;
		for(int i = 0; i < MAX_MSG_SIZE; i++) {
			buffer[i] = other.buffer[i];
		}
	}

	// Assignment operator
	packet_t& operator=(const packet_t& other) {
		// Is this self assignment?
		if(this != &other) {
			// No, copy over data
			ID = other.ID;
			senderID = other.senderID;
			misCounter = other.misCounter;
			misCounter2 = other.misCounter2;
			misCounter3 = other.misCounter3;
			packetType = other.packetType;
			for(int i = 0; i < MAX_MSG_SIZE; i++) {
				buffer[i] = other.buffer[i];
			}
		}

	    return *this;
	}

	// Copy the contents the other packet into this packet
	void copy(const packet_t& other) {
		ID = other.ID;
		senderID = other.senderID;
		packetType = other.packetType;
		misCounter = other.misCounter;
		misCounter2 = other.misCounter2;
		misCounter3 = other.misCounter3;
		for(int i = 0; i < MAX_MSG_SIZE; i++) {
			buffer[i] = other.buffer[i];
		}
	}
};
