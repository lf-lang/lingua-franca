#pragma once

class PingMessage {
private:
	const int mPingsLeft;
public:
	PingMessage(const int pingsLeft):
		mPingsLeft(pingsLeft) {}

	bool hasNext() const {
		return mPingsLeft > 0;
	}

	int getPingsLeft() const {
		return mPingsLeft;
	}

	reactor::ImmutableValuePtr<PingMessage> next() const {
		return reactor::make_immutable_value<PingMessage>(mPingsLeft-1);
	}
};
