#pragma once

enum MsgType {
	StartMsg,
	HungryMsg,
	DoneMsg,
	ExitMsg,
	DeniedMsg,
	EatMsg
};

struct Message {

	MsgType type;
};
