# Makefile for MQTTDistributed.lf
compile:	Distributed.lf
	lfc --no-compile Distributed.lf
	gcc -O2 src-gen/Distributed_Sender.c -o bin/Distributed_Sender -pthread
	gcc -O2 src-gen/Distributed_Receiver.c -o bin/Distributed_Receiver -pthread
	gcc -O2 src-gen/Distributed_RTI.c -o bin/Distributed_RTI -pthread

run:	bin/Distributed_RTI bin/Distributed_Sender bin/Distributed_Receiver
	bin/Distributed_RTI &
	bin/Distributed_Sender &
	bin/Distributed_Receiver

all:	compile run