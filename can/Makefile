CC ?= gcc

all: uio-can-local-loopback

uio-can-local-loopback: uio-can-example.c
	$(CC) -o uio-can-local-loopback uio-can-example.c -Wall -Wpedantic

clean:
	rm -rf uio-can-local-loopback .*.swp .*.un* *~
