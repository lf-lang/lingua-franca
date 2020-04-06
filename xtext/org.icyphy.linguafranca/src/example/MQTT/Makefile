# Makefile for MQTTDistributed.lf
compile:	MQTTDistributed.lf MQTTPublisher.lf MQTTSubscriber.lf
	lfc --no-compile MQTTDistributed.lf
	gcc -I/usr/local/include -L/usr/local/lib -g -lpaho-mqtt3c src-gen/MQTTDistributed_Publisher.c -o bin/MQTTDistributed_Publisher -pthread
	gcc -I/usr/local/include -L/usr/local/lib -g -lpaho-mqtt3c src-gen/MQTTDistributed_Subscriber.c -o bin/MQTTDistributed_Subscriber -pthread
	gcc -O2 src-gen/MQTTDistributed_RTI.c -o bin/MQTTDistributed_RTI -pthread

run:	bin/MQTTDistributed_RTI bin/MQTTDistributed_Publisher bin/MQTTDistributed_Subscriber
	bin/MQTTDistributed_RTI &
	bin/MQTTDistributed_Publisher &
	bin/MQTTDistributed_Subscriber
	
all:	compile run
