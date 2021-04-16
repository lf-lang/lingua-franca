
#include "NQueensCommon.hh"



int NUM_WORKERS = 20;
int SIZE = 12;
int THRESHOLD = 4;
int PRIORITIES = 10;
int SOLUTIONS_LIMIT = 1500000;

// solutions to validate results
const long SOLUTIONS[] = {
            1,
            0,
            0,
            2,
            10,     /* 5 */
            4,
            40,
            92,
            352,
            724,    /* 10 */
            2680,
            14200,
            73712,
            365596,
            2279184, /* 15 */
            14772512,
            95815104,
            666090624,
            4968057848L,
            39029188884L, /* 20 */
};


bool boardValid(int n, std::vector<int> a) {

	int i, j;
	int p, q;

	for(i = 0; i < n; ++i) {
		p = a[i];

		for(j = (i + 1); j < n; ++j) {
			q = a[j];
			if(q == p || q == p - (j - i) || q == p + (j - i)) {
				return false;
			}
		}
	}
	return true;
}


// Searches for results recursively and return the number of found
// solutions.
int nqueensKernelSeq(std::vector<int> a, int depth, int size) {

	if(size == depth) {
		return 1;
	}

	int numberOfSolutionsFound = 0;
	std::vector<int> b;
	b.reserve(depth + 1);

	int i = 0;
	while(i < size) {
		b.insert(begin(b), begin(a), begin(a) + depth);
		b[depth] = i;
		if(boardValid(depth + 1, b)) {
			numberOfSolutionsFound += nqueensKernelSeq(b, depth + 1, size);
		}
		i += 1;
	}
	return numberOfSolutionsFound;
}

WorkMessage::WorkMessage(int _priority, std::vector<int> _data, int _depth):
						data(_data), depth(_depth) {
	priority = std::min(PRIORITIES -1, std::max(0, _priority));
}
