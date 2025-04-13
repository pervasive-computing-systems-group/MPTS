/*
 * P2P.h
 *
 * Created by:	Jonathan Diller
 * On: 			January 14, 2024
 *
 * Description: peer-to-peer message system. Used to validate socket programming logic.
 *
 */

#pragma once

#include "defines.h"
#include "Node.h"


class P2P : public Node {
public:
	P2P(int nodeID);
	~P2P();

private:
	// Starts the message passing logic
	void StartAlgo();
	// Runs the message passing logic
	void RunAlgo(packet_t* packet);
	// The node doesn't know what algorithm we are running and must ask the child
	int GetAlgo() {return 0;}
};
