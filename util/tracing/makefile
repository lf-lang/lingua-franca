# Makefile for utilities that convert Lingua Franca trace files
# into other formats.
# @author: Edward A. Lee

CC=gcc
CFLAGS=-I../../xtext/org.icyphy.linguafranca/src/lib/core/
DEPS=

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

trace_to_csv: trace_to_csv.o
	$(CC) -o trace_to_csv trace_to_csv.o
	
install: trace_to_csv
	mv trace_to_csv ../../bin
	
clean:
	rm *.o

 
