###############################################################################
# CS/CNS 171 Fall 2015
#
# This is a template Makefile for the assignment. Edit it however you find
# convenient.
###############################################################################

CC = g++
FLAGS = -g
INCLUDE = -I../

SOURCES = pt4.cpp  

EXENAME = pt4

all: $(EXENAME)

$(EXENAME): $(SOURCES)
	@$(CC) $(INCLUDE) $(FLAGS) $(SOURCES) -o $(EXENAME)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean


