CXX = g++
FLAGS = -Wall -Werror -std=c++11 -llcm
BIN_PATH = ./bin
OBJ_PATH = ./obj
SRC_PATH = ./src
TEST_PATH = ./unit_tests

all: test GridTest Mapper

obj/%.o: $(SRC_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

obj/%.o: $(TEST_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

Mapper: $(OBJ_PATH)/Mapper.o

GridTest: $(OBJ_PATH)/GridMap.o $(OBJ_PATH)/GridTests.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/GridTest

test: GridTest
	./bin/GridTest >/dev/null

clean:
	rm bin/*
	rm obj/*
