#include <iostream>

#include "defines.h"
#include "P2P.h"
#include "GradientSolver.h"
#include "HungGSSolver.h"


#define DEFAULT_INPUT "masp-exmp.txt"

// main function
int main(int arg, char const *argv[]) {
	int peerID;
	int algo;
	const char* inputFile;

	// Parse inputs
	if(arg == 3) {
		peerID = atoi(argv[1]);
		algo = atoi(argv[2]);
		inputFile = DEFAULT_INPUT;
	}
	else if(arg == 4) {
		peerID = atoi(argv[1]);
		algo = atoi(argv[2]);
		inputFile = argv[3];
	}
	else {
		fprintf(stderr, "[ERROR] : expected input: <node-ID> <algorithm> [input-file]\n e.g.: ./assignTasks 0 1 masp-exmp.txt\n");
		exit(1);
	}

	printf("Problem setup: Algorithm = %d, Peer-ID = %d, input = %s\n", algo, peerID, inputFile);

	switch(algo) {
	case e_Algo_P2P: {
		// Create a basic distributed node and run it
		P2P p2p(peerID);
		p2p.Run();
	}
	break;

	case e_Algo_GS: {
		// Create a basic distributed node and run it
		GradientSolver gsSolver(peerID, inputFile);
		printf("Running GS Algorithm\n");
		gsSolver.Run();
	}
	break;

	case e_Algo_HUNGGS: {
		// Create a basic distributed node and run it
		HungGSSolver gsSolver(peerID, inputFile);
		printf("Running Hungarian+GS Algorithm\n");
		gsSolver.Run();
	}
	break;

	default:
		printf("\nUnknown Algorithm!\n\n");
	}


	return 0;
}
