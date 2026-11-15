#
# Simple Makefile for compiling programs for initio robot car 
#
SHELL	= bash
GCC	= gcc
CFLAGS	= -Wall -Werror -mfloat-abi=hard
LFLAGS	= -linitio -lcurses -lwiringPi -lpthread

PROG 	= line_follower

.PHONY: all run help

all: $(PROG)

run: $(PROG)
	./$<

schedule: $(PROG)
	rtcs_schedule $(PROG)

# Compile without linking (useful for testing on other systems)
test_compile:
	$(GCC) -c -o $(PROG).o -Wall -Werror -I./include $(PROG).c

% : %.c
	$(GCC) -o $@ $(CFLAGS) $(LFLAGS) $<

clean:
	rm -f $(PROG) $(PROG).o

help:
	@echo
	@echo "Possible commands:"
	@echo " > make run"
	@echo " > make schedule"
	@echo " > make test_compile"
	@echo " > make clean"
	@echo

