CC=gcc
LDFLAGS = $(shell python3-config --libs) 
CFLAGS = $(shell python3-config --includes)

%.o: %.c
	$(CC) -c -fPIC -o $@ $< $(CFLAGS)

all: test.o
	$(CC) test.o -shared -o test.so $(LDFLAGS)
	rm -f *.o

clean:
	rm -f test.o test
