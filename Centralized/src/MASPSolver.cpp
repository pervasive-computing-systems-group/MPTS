#include "MASPSolver.h"

MASPSolver::MASPSolver() {
}

MASPSolver::~MASPSolver() {}

// Sets a = min(a, b), returns true if b < a
bool MASPSolver::ckmin(double* a, double b) {
	if(b < *a) {
		*a = b;
		return true;
	}
	else {
		return false;
	}
}
