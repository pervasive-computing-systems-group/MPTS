# Distributed Task Assignments
This repository holds the distributed task assignment algorithms described in the publication "*Failure-Aware Distributed Task Assignments for Teams of Drones*."

### To Run:
From the directory where the README is located, run the following commands

 1. To build the project:

From the terminal, run: 

	mkdir build
	cd build
	cmake ..
	make

This should build the entire project. After initially building, you can simply run 'make' again from within the build directory to build the project again.	

2. To run each peer:

Update the IP address and port number of each node in the inc/defines.h file. Make sure that each node has a different IP/port number combination.
Open a new terminal for each different node. In each terminal, run: ./assignTasks <node-ID> <algorithm>

e.g.: `./assignTasks 0 0`

In this example, we are launching node 0 and running algorithm 0 - messaging passing.
	
### How it works:
The auctioning algorithm is currently not implemented yet. The code uses multithreading to create a thread for the server (to receive data packets) and can run a client (to send data packets). Sending packets back and forth is based on the peer table defined in the inc/defines.h file.

### Known bug/limitation:
- The peer table is currently hardcoded in inc/defines.h. This should be changed to something that takes in a file at runtime with the peer table in the file.
- Need some type of logic that triggers each algorithm but waits until all nodes are already launched. Perhaps assume that node 0 starts the algorithm?
	

