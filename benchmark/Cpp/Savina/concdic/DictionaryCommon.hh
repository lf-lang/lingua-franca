#pragma once

enum MsgType {
	DoWorkMsg,
	EndWorkMsg,
	ResultMsg,
	ReadMsg,
	WriteMsg
};

struct Message {

	MsgType type;
	int key;
	int value;

	Message() {}

	Message(MsgType _type, int _key, int _value):
		type(_type), key(_key), value(_value) {}

	Message(MsgType _type):
		type(_type) {}

	Message(MsgType _type, int _key):
		type(_type), key(_key) {}
};
