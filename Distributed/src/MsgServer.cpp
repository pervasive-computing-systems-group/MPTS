#include "MsgServer.h"

MsgServer::MsgServer(int prID) {
	if(SANITY_PRINT)
		printf("[SRVR] Starting Message Server\n");
	nodeID = prID;
	bExit = false;
	bRunning = false;

    // Start server thread
    pthread_create(&server, NULL, &MsgServer::server_helper, this);
    while(!bRunning) {
    	// Do nothing...
    	std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

MsgServer::~MsgServer() {
	// wait for thread to complete
	pthread_join(server, NULL);
}

void* MsgServer::server_helper(void *context) {
	return ((MsgServer *)context)->server_func();
}

void* MsgServer::server_func() {
	if(DEBUG_MSG_SERV)
		printf("server portion running.\n");

	int server;
	bExit = false;
	packet_t message;

	struct sockaddr_in server_addr;
	socklen_t size;

	/* ---------- ESTABLISHING SOCKET CONNECTION ----------*/
	/* --------------- socket() function ------------------*/

	server = socket(AF_INET, SOCK_STREAM, 0);

	if (server < 0) {
		fprintf(stderr, "[ERROR] MsgServer::server : Error establishing socket...\n");
		exit(1);
	}
	
	int reuse = 1;
    	if (setsockopt(server, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0)
        	perror("setsockopt(SO_REUSEADDR) failed");

	#ifdef SO_REUSEPORT
    	if (setsockopt(server, SOL_SOCKET, SO_REUSEPORT, (const char*)&reuse, sizeof(reuse)) < 0) 
       	 perror("setsockopt(SO_REUSEPORT) failed");
	#endif

	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htons(INADDR_ANY);
	server_addr.sin_port = htons(peer_table[nodeID].nPort);

	/* ---------- BINDING THE SOCKET ---------- */
	/* ---------------- bind() ---------------- */


	if ((bind(server, (struct sockaddr*)&server_addr,sizeof(server_addr))) < 0) {
		fprintf(stderr, "[ERROR] MsgServer::server : Error binding connection, the socket has already been established...\n");
		exit(1);
	}

	size = sizeof(server_addr);

	/* ------------- LISTENING CALL ------------- */
	/* ---------------- listen() ---------------- */

	listen(server, 5); // Increase the backlog to allow multiple clients to wait in the queue

	fd_set activeSockets;
	int nClientSocket;

	FD_ZERO(&activeSockets);
	FD_SET(server, &activeSockets);

	// The server is now running
	bRunning = true;

	while(!bExit) {
		// Wait for incoming connection requests
		nClientSocket = accept(server, (struct sockaddr *)&server_addr, &size);

		// Verify we connected
		if(nClientSocket == -1) {
			if(DEBUG_MSG_SERV)
				printf("Failed to accept connection request");
			continue;
		}
		else {
			if(DEBUG_MSG_SERV)
				printf("Connected to server\n");
		}

		// Read from server
		int nBytesRead = read(nClientSocket, &message, sizeof(packet_t));

		if(nBytesRead >= 0) {
			// Lock the queue...
			const std::lock_guard<std::mutex> lock(m_PacketQMutex);
			// Add to packet queue
			qPacketQueue.push(message);
		}
		else {
			if(DEBUG_MSG_SERV)
				printf("Failed to read bytes...\n");
		}

		if(DEBUG_MSG_SERV)
			puts("Client handled!");
		close(nClientSocket);
	}

	if(SANITY_PRINT)
		printf("[SRVR] Server has exited.\n");
	return 0;
}

// Check for next packet on packet queue. Returns true if a packet was retrieved, false otherwise.
bool MsgServer::GetNextPacket(packet_t* packet) {
	// Lock the queue...
	const std::lock_guard<std::mutex> lock(m_PacketQMutex);
	// Verify that there is a packet to pop
	if(!qPacketQueue.empty()) {
		// Get packet and copy it into packet
		packet->copy(qPacketQueue.front());
		// Pop packet off of queue
		qPacketQueue.pop();

		return true;
	}
	else {
		// No messages to pop...
		return false;
	}
}



















