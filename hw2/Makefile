CC = g++
FLAGS = -g

INCLUDE = -I./include/
SOURCES = $(wildcard src/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

EXENAME = main

all: $(EXENAME)

$(EXENAME): $(OBJECTS)
	@$(CC) $(INCLUDE) $(FLAGS) $(OBJECTS) -o $(EXENAME)

%.o: %.cpp
	@$(CC) $(INCLUDE) $(FLAGS) -c $< -o $@

clean:
	rm -f src/*.o $(EXENAME)

.PHONY: all clean
