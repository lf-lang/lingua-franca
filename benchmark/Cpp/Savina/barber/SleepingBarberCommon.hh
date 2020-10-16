#pragma once

enum MsgType {
	ResetMsg,
	StartMsg,
	EnterMsg,
	NextMsg,
	ExitMsg,
	FullMsg,
	WaitMsg,
	ReturnedMsg,
	DoneMsg
};

struct Message {

	MsgType type;
	int id;

	Message(MsgType _type):
		type(_type) {}

	Message(MsgType _type, int _id):
		type(_type), id(_id) {}
};
