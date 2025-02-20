CXX := g++
CXXFLAGS := -std=c++20 -O3 -Wall -Wextra -MMD -MP -Iinclude

SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
SOURCES := $(wildcard $(SRC_DIR)/*.cpp) main.cpp
OBJECTS := $(patsubst %.cpp, $(BUILD_DIR)/%.o, $(SOURCES))
DEPFILES := $(patsubst %.o, %.d, $(OBJECTS))
TARGET := purepursuit

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)

-include $(DEPFILES)

$(BUILD_DIR)/%.o: %.cpp
	mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

.PHONY: clean

