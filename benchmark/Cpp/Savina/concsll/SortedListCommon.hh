#pragma once

enum MsgType {
	EndWorkMsg,
	ResultMsg,
	SizeMsg,
	ContainsMsg,
	WriteMsg
};

struct Message {

	MsgType type;
	int value;
};
