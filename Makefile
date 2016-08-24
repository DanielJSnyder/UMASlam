CXX = g++
FLAGS = -Wall -Werror -Wextra -std=c++11 -llcm
BIN_PATH = ./bin
OBJ_PATH = ./obj
SRC_PATH = ./src
TEST_PATH = ./unit_tests

all: test

obj/%.o: $(SRC_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

obj/%.o: $(TEST_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

GridTest: $(OBJ_PATH)/GridMap.o $(OBJ_PATH)/GridTests.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/GridTest

test: GridTest
	./bin/GridTest >/dev/null

clean:
	rm bin/*
	rm obj/*
