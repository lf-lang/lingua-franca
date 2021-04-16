#pragma once

#include  <vector>

extern std::vector<std::vector<double>> * H;
extern std::vector<std::vector<double>> * F;

enum MsgType {
	NextMsg,
	ExitMsg,
	BootMsg
};

struct Message {
	MsgType type;
};
