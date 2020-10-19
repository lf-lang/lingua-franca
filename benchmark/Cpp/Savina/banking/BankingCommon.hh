#pragma once

enum MsgType {
	StartMsg,
	StopMsg,
	ReplyMsg,
	DebitMsg,
	CreditMsg
};

struct Message {

	MsgType type;
	double amount;
	int recipient;

	Message(MsgType _type, double _amount, int _recipient):
		type(_type), amount(_amount), recipient(_recipient) {}

	Message() {}

	Message(MsgType _type):
		type(_type) {}
};
