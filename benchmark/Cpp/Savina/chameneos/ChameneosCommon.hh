#pragma once


enum Color {
	RED,
	YELLOW,
	BLUE,
	FADED
};

enum MsgType {
	StartMsg,
	MeetMsg,
	ChangeMsg,
	MeetingCountMsg,
	ExitMsg
};

struct Message {
	MsgType type;
	Color color;
	int id;

	Message() {}

	Message(MsgType _type, Color _color, int _id):
		type(_type), color(_color), id(_id) {}

	Message(MsgType _type):
		type(_type) {
		// use default values for other members
	}
};
