#include "P2P.h"

P2P::P2P(int nodeID) : Node(nodeID) {

}

P2P::~P2P() {}

// Starts the message passing logic
void P2P::StartAlgo() {
	// Trick each node into running our algorithm
	packet_t dummy;
	dummy.packetType = E_PacketType::e_MESSAGING_PCKT;
	// Tell each node to start..
	for(int i = 0; i < NUM_AGENTS; i++) {
		m_msgSender.SendMsg(i, &dummy);
	}
}

void P2P::RunAlgo(packet_t* packet) {
	// Ignore incoming packet - it was a dummy packet. Run message passing logic
	packet_t message;

	// While the user does not shutdown this node...
	bool runAgain = true;
	while(runAgain) {
		char input[64];

		printf("Please enter peer ID to send message, 'm' to read message queue, or '#' to exit: ");
		std::cin.getline(input, 64); // Read the entire line including spaces

		if(input[0] == '#') {
			// Shut-down...
			runNode = false;
			runAgain = false;
		}
		else if(input[0] == 'm') {
			// Get messages off of message queue
			if(m_msgServer.GetNextPacket(&message)) {
				printf("Got message off queue. Message ID = %d\n -%s\n",
						message.ID, message.buffer);
			}
			else {
				printf("No message found..\n");
			}
		}
		else {
			int portNum = atoi(input);

			// Get user input
			printf("Please write message to peer: ");
			std::cin.getline(message.buffer, MAX_MSG_SIZE);
			message.packetType = E_PacketType::e_MESSAGING_PCKT;

			// Send over the message
			printf("Sending message to %d\n", portNum);
			m_msgSender.SendMsg(portNum, &message);
		}
	}
}
