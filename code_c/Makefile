# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Distributed as part of the University of South Carolina CSCE491 course
# materials. Please do not redistribute without written authorization.

C_UTIL_DIR=$(shell pwd)/../utils/c_utils
CC=gcc
CFLAGS=-Wall -Wextra -pedantic -Wmisleading-indentation -Werror -std=gnu11 -g -Og
CINCLUDE=-lm -I$(C_UTIL_DIR)


sources = $(wildcard *.c)
headers = $(wildcard *.h)
objects = $(patsubst %.c,%.o,$(sources))

a.out: $(objects) waves.o
	$(CC) $(CFLAGS) $^ $(CINCLUDE) -o $@

%.o: %.c $(headers)
	$(CC) $(CFLAGS) -c $< -o $@ $(CINCLUDE)

waves.o:
	$(MAKE) CFLAGS="$(CFLAGS)" CINCLUDE="$(CINCLUDE)" CC="$(CC)" -C $(C_UTIL_DIR)
	cp $(C_UTIL_DIR)/waves.o ./

clean:
	rm -f $(objects) waves.o a.out
	$(MAKE) -C $(C_UTIL_DIR) clean
