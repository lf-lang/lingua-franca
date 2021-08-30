#pragma once

enum MsgType {
	NextMsg,
	ExitMsg,
	BootMsg
};

struct Message {
	MsgType type;
};
