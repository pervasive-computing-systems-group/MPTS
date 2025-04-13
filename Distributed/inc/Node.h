/*
 * Node.h
 *
 * Created by:	Jonathan Diller
 * On: 			January 14, 2024
 *
 * Description: Distributed node base class
 *
 */

#pragma once

#include "defines.h"
#include "MsgServer.h"
#include "MsgSender.h"

#define PRINT_RESULTS	1

class Node {
public:
	Node(int nodeID);
	virtual ~Node();

	// Universal run function
	void Run();

protected:
	/*
	 * Initiates the distrusted algorithm implemented in any derived class.
	 * Only node 0 will run this function and we assume that node 0 is the
	 * node that initiates any algorithm.
	 */
	virtual void StartAlgo() = 0;
	// Runs the distrusted algorithm implemented in any derived class
	virtual void RunAlgo(packet_t* packet) = 0;
	// Function that forces the packet server to shut-down
	void ShutDown();

	// Packet server
	MsgServer m_msgServer;
	// Packet sender member object
	MsgSender m_msgSender;
	// This node's ID
	int mNodeId;
	bool runNode = true;

private:
};
