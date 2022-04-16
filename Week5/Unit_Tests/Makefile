UNAME=$(shell uname)

CCFLAGS=-Wall -O3 -std=gnu99
CC=gcc

all: cutpointtests

cutpointtests: main.o queue.o cuttingpoint1.o ctest.h unittests.o
	$(CC) $(LDFLAGS) main.o queue.o cuttingpoint1.o unittests.o -o cutpointtests

remake: clean all

%.o: %.c ctest.h
	$(CC) $(CCFLAGS) -c -o $@ $<

clean:
	rm -f cutpointtests *.o
