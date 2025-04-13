/*
 * MsgSender.h
 *
 * Created by:	Yee Shen Teoh, Jonathan Diller
 * On: 			January 12, 2024
 *
 * Description: Packet sender class that is used to send data packets to
 * other nodes.
 *
 */

#pragma once

#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>

#include "defines.h"

#define DEBUG_MSG_SEND	DEBUG || 0

class MsgSender {
public:
	MsgSender(int nodeID);
	~MsgSender();

	// Sends a message to the indicated peer
	bool SendMsg(int peerID, packet_t* message);

private:
	int mNodeID;
};
