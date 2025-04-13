#include "Node.h"

Node::Node(int nodeID) : m_msgServer(nodeID),  m_msgSender(nodeID) {
	mNodeId = nodeID;
	runNode = true;
}

Node::~Node() {}


void Node::Run() {
	packet_t packet;

	// Tell child to start running
	StartAlgo();

	// Run until we hit a shut-down triggering event
	while(runNode) {
		// Check for incoming packets
		bool processPacket = m_msgServer.GetNextPacket(&packet);
		// If there is a packet..
		if(processPacket) {
			// Check the packet's type
			if(packet.packetType > E_PacketType::e_MAX_MAINT_PCKT) {
				// This is an algorithm packet.. run derived algorithm
				RunAlgo(&packet);
			}
			else {
				// We have a maintenance packet...
				switch(packet.packetType) {
				case E_PacketType::e_NULL_PCKT:
					// Do nothing...
					if(SANITY_PRINT)
						printf("[NODE] Received NULL packet\n");
					break;

				case E_PacketType::e_SHUT_DOWN_PCKT:
					// Start shut-down
					runNode = false;
					break;

				default:
					// Do nothing...
					if(SANITY_PRINT)
						printf("[NODE] Received unknown packet type = %d\n", packet.packetType);
				}
			}
		}

		// Run node maintenance
	}

	// Shutdown our packet server
	ShutDown();
}

void Node::ShutDown() {
	// Tell the server to shutdown
	m_msgServer.ExitServer();
	// Wake the server up so it knows we told it to shut down
	packet_t dummy;
	m_msgSender.SendMsg(mNodeId, &dummy);
}
