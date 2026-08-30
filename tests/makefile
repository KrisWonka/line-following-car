CC = gcc

CFLAGS = -Wall -O2

all: runner

runner: runner.c ../src/image.c
	$(CC) $(CFLAGS) runner.c ../src/image.c -I../src -o runner -lm

clean:
	rm -f runner
