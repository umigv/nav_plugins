CXX := g++
CXXFLAGS := -std=c++20 -O3 -Wall -Wextra -MMD -MP -I../include -Iinclude

SRC_DIRS := ../src ./src
BUILD_DIR := build
TARGET := pure_pursuit

SOURCES := $(filter-out ../src/pure_pursuit_controller.cpp, $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.cpp)) main.cpp)
OBJECTS := $(patsubst %,%,$(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES:.cpp=.o))))
DEPFILES := $(OBJECTS:.o=.d)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)

-include $(DEPFILES)

$(BUILD_DIR)/%.o: ../src/%.cpp
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: ./src/%.cpp
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

.PHONY: clean