#include "MsgSender.h"

MsgSender::MsgSender(int nodeID) {
	if(SANITY_PRINT)
		printf("[MSG SNDR] Starting Message Sender\n");
	mNodeID = nodeID;
}

MsgSender::~MsgSender() {}


// Sends a message to the indicated peer
bool MsgSender::SendMsg(int peerID, packet_t* message) {
	int client;
	struct sockaddr_in server_addr;

	client = socket(AF_INET, SOCK_STREAM, 0);

	/* ---------- ESTABLISHING SOCKET CONNECTION ----------*/
	/* --------------- socket() function ------------------*/

	if (client < 0) {
		fprintf(stderr, "[ERROR] MsgSender::SendMsg : establishing socket...\n");
		exit(1);
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(peer_table[peerID].nPort);

	if (inet_pton(AF_INET, peer_table[peerID].sIP, &(server_addr.sin_addr)) <= 0) {
		fprintf(stderr, "[ERROR] MsgSender::SendMsg : Invalid address/ Address not supported\n");
		close(client);
		exit(1);
	}

	/* ---------- CONNECTING THE SOCKET ---------- */
	/* ---------------- connect() ---------------- */

	if (connect(client,(struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
		if(SANITY_PRINT)
			printf("[WARNING] MsgSender::SendMsg : connection not successful\n");
		close(client);
		// Return false, don't try to send the packet
		return false;
	}

	// Once it reaches here, the client can send a message first.
	if(DEBUG_MSG_SEND)
		printf("=> Connection confirmed\n");
	// Stamp node with our ID
	message->senderID = mNodeID;
	// Send packet
	int byteSent = send(client, message, sizeof(packet_t), 0);
	if(DEBUG_MSG_SEND)
		printf("   Sent %d bytes to peer\n", byteSent);

	/* ---------------- CLOSE CALL ------------- */
	/* ----------------- close() --------------- */

	close(client);
	return (byteSent >= 0);
}
