#pragma once

enum MsgType {
	ResetMsg,
	ComputeMsg,
	ResultMsg,
	NextTermMsg,
	GetTermMsg
};

struct Message {

	MsgType type;
	double term;
};
