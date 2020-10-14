#pragma once

class PseudoRandom {
private:
	long mValue;

public:
	PseudoRandom(long value):
		mValue(value) {}

	PseudoRandom():
		mValue(74755) {}

	long nextLong() {
		mValue = ((mValue * 1309) + 13849) & 65535;
		return mValue;
	}

	int nextInt() {
		return static_cast<int>(nextLong());
	}

	double nextDouble() {
		return 1.0 / (nextLong() + 1);
	}

	int nextInt(int exclusive_max) {
		return nextInt() % exclusive_max;
	}
};

enum MsgType {
	StartMsg,
	PingMsg,
	PongMsg,
	ExitMsg
};
