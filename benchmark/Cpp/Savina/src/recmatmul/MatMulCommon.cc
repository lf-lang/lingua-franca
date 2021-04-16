
#include <cmath>
#include "MatMulCommon.hh"
#include "reactor-cpp/logging.hh"


std::vector<std::vector<double>> * A = nullptr;
std::vector<std::vector<double>> * B = nullptr;
std::vector<std::vector<double>> * C = nullptr;

const double EPSILON = 0.0001;

WorkMessage::WorkMessage(int _priority, int _srA, int _scA, int _srB,
		int _scB, int _srC, int _scC, int _numBlocks, int _dim):
						priority(_priority), srA(_srA), scA(_scA), srB(_srB),
						scB(_scB), srC(_srC), scC(_scC), numBlocks(_numBlocks), dim(_dim) {
}

void initializeData(int dataLength) {

	if(A) delete A;
	if(B) delete B;
	if(C) delete C;

	A = new std::vector<std::vector<double>>(dataLength, std::vector<double>(dataLength));
	B = new std::vector<std::vector<double>>(dataLength, std::vector<double>(dataLength));
	C = new std::vector<std::vector<double>>(dataLength, std::vector<double>(dataLength));

	for (int i = 0; i < dataLength; ++i) {
		for (int j = 0; j < dataLength; ++j) {
			A->at(i)[j] = i;
			B->at(i)[j] = j;
		}
	}
}

bool valid(int dataLength) {

	for (int i = 0; i < dataLength; i++) {
		for (int j = 0; j < dataLength; j++) {
			double actual = C->at(i)[j];
			double expected = 1.0 * dataLength * i * j;
			if (!fabs(actual-expected) < EPSILON) {
				reactor::log::Info() << "Validation failed for (i,j)=" << i << "," << j << " with (" << actual << "," << expected << ")";
				return false;
			}
		}
	}
	return true;
}
