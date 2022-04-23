UNAME=$(shell uname)

CCFLAGS=-Wall -O3 -std=gnu99
CC=gcc

all: fcfs

fcfs: main.o queue.o fcfs.o ctest.h fcfstests.o
	$(CC) $(LDFLAGS) main.o queue.o fcfs.o fcfstests.o -o firstcomefirstserved

remake: clean all

%.o: %.c ctest.h
	$(CC) $(CCFLAGS) -c -o $@ $<

clean:
	rm -f firstcomefirstserved *.o
