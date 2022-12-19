# Makefile for utilities that convert Lingua Franca trace files
# into other formats.
# @author: Edward A. Lee

CC=gcc
CFLAGS=-I../../org.lflang/src/lib/c/reactor-c/include/core/ \
		-I../../org.lflang/src/lib/c/reactor-c/include/core/modal_models \
		-I../../org.lflang/src/lib/c/reactor-c/include/core/platform \
		-I../../org.lflang/src/lib/c/reactor-c/include/core/utils \
		-Wall
DEPS=
LIBS=-lcurl

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

trace_to_csv: trace_to_csv.o trace_util.o
	$(CC) -o trace_to_csv trace_to_csv.o trace_util.o
	
trace_to_chrome: trace_to_chrome.o trace_util.o
	$(CC) -o trace_to_chrome trace_to_chrome.o trace_util.o

trace_to_influxdb: trace_to_influxdb.o trace_util.o
	$(CC) -o trace_to_influxdb trace_to_influxdb.o trace_util.o $(LIBS)

install: trace_to_csv trace_to_chrome trace_to_influxdb
	mv trace_to_csv ../../bin
	mv trace_to_chrome ../../bin
	mv trace_to_influxdb ../../bin
	
clean:
	rm -f *.o
