#pragma once

enum MsgType {
	DataItemMsg,
	ProduceDataMsg,
	ProducerExitMsg,
	ConsumerAvailableMsg,
	ConsumerExitMsg
};

struct Message {

	MsgType type;
	double data;
};
