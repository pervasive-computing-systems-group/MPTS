/*
 * MsgServer.h
 *
 * Created by:	Yee Shen Teoh, Jonathan Diller
 * On: 			January 12, 2024
 *
 * Description: Packet server class that starts a server on a new thread and
 * stores incoming packets in a queue.
 *
 */

#pragma once

#include <queue>
#include <unistd.h>
#include <netdb.h>
#include <mutex>
#include <thread>
#include <chrono>

#include "defines.h"

#define DEBUG_MSG_SERV	DEBUG || 0

class MsgServer {
public:
	MsgServer(int prID);
	~MsgServer();

	// Runs message server
	void* server_func();
	// Used to launch the message server in a new thread
	static void* server_helper(void *context);
	// Check for next packet on packet queue. Returns true if a packet was retrieved, false otherwise.
	bool GetNextPacket(packet_t* packet);
	// Tells the server to exit - may not actually shut-down server!
	void ExitServer() {bExit=true;}

private:
	// pthread to run server
    pthread_t server;
    // Packet queue
	std::queue<packet_t> qPacketQueue;
	// Mutex to protect packet queue
	std::mutex m_PacketQMutex;
	// Our ID
	int nodeID;
	// Server exit flag
	bool bExit;
	// Server running flag
	bool bRunning;
};
