CXX = g++
FLAGS = -Wall -Wno-unused-function -Werror -std=c++11
OPTIMIZATION_FLAGS = -O3
PFLAGS = -pg -DPROFILE -O3
SFML_FLAGS = -lsfml-graphics -lsfml-window -lsfml-system
LCM_FLAGS = -llcm
BIN_PATH = ./bin
OBJ_PATH = ./obj
SRC_PATH = ./src
TEST_PATH = ./unit_tests

all: GridTest MapperTest Localizer SlamTest PointCloudTest PointCloudPrinter ParticlePrinter PointCloudVis

optimized: FLAGS += $(OPTIMIZATION_FLAGS)
optimized: all

debug: FLAGS += -DSLAM_DEBUG_LEVEL=$(LEVEL) -g3
debug: all

profile: FLAGS += $(PFLAGS)
profile: all

obj/%.o: $(SRC_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

obj/%.o: $(TEST_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

obj/MapDrawer.o: $(SRC_PATH)/MapDrawer.cpp
	$(CXX) $(FLAGS) $(SFML_FLAGS) -c $^ -o $@

Localizer: $(OBJ_PATH)/Localizer.o

Slam: $(OBJ_PATH)/SLAM.o

SlamTest: $(OBJ_PATH)/SLAM.o $(OBJ_PATH)/Localizer.o $(OBJ_PATH)/MapDrawer.o $(OBJ_PATH)/Mapper.o $(OBJ_PATH)/GridMap.o $(OBJ_PATH)/slamtest.o $(OBJ_PATH)/CoordTransformer.o $(OBJ_PATH)/FakeCompass.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/SlamTest $(SFML_FLAGS) $(LCM_FLAGS)

MapperTest: $(OBJ_PATH)/Mapper.o $(OBJ_PATH)/MapDrawer.o $(OBJ_PATH)/MapperTester.o $(OBJ_PATH)/GridMap.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/MapperTest $(SFML_FLAGS) $(LCM_FLAGS)

GridTest: $(OBJ_PATH)/GridMap.o $(OBJ_PATH)/GridTests.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/GridTest $(LCM_FLAGS)

PointCloudTest: $(OBJ_PATH)/PointCloud.o $(OBJ_PATH)/point_cloud_test.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/pointcloudmaker $(LCM_FLAGS)

PointCloudPrinter: $(OBJ_PATH)/PointCloudPrinter.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/PointCloudPrinter $(LCM_FLAGS)

PointCloudVis: 
	$(CXX) $(FLAGS) src/PointCloudVisualizer.cpp  -o $(BIN_PATH)/PointCloudVis $(LCM_FLAGS) $(SFML_FLAGS)

ParticlePrinter: $(OBJ_PATH)/ParticlePrinter.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/ParticlePrinter $(LCM_FLAGS)

test: GridTest MapperTest
	./bin/GridTest >/dev/null

report:
	$(MAKE) -C report 

clean:
	- rm bin/*
	- rm obj/*
