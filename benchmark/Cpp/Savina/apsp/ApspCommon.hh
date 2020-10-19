#pragma once

using MatrixOfLong = std::vector<std::vector<long>>;

struct ApspResultMessage {
	int k;
	int myBlockId;
	reactor::ImmutableValuePtr<MatrixOfLong> initData;

	ApspResultMessage() {}

	ApspResultMessage(int _k, int _myBlockId, reactor::ImmutableValuePtr<MatrixOfLong> _initData):
		k(_k), myBlockId(_myBlockId), initData(_initData) {}
};
