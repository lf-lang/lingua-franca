
#include "NQueensCommon.hh"

size_t NUM_WORKERS = 20;
size_t SIZE = 12;
size_t THRESHOLD = 4;
size_t PRIORITIES = 10;
size_t SOLUTIONS_LIMIT = 1500000;

// solutions to validate results
const size_t SOLUTIONS[] = {
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


bool boardValid(size_t n, const std::vector<size_t>& a) {
	size_t p{0};
	size_t q{0};

	for(size_t i{0}; i < n; ++i) {
		p = a[i];
		for(size_t j{i + 1}; j < n; ++j) {
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
size_t nqueensKernelSeq(const std::vector<size_t>& a, size_t depth, size_t size) {
	if(size == depth) {
		return 1;
	}

	size_t numberOfSolutionsFound{0};
	std::vector<size_t> b;
	b.reserve(depth + 1);

	size_t i{0};
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

WorkMessage::WorkMessage(size_t _priority, const std::vector<size_t>& _data, size_t _depth):
						data(_data), depth(_depth) {
	priority = std::min(PRIORITIES - 1, std::max(0ul, _priority));
}
