CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -Iinclude
LDFLAGS ?= -lpthread

SRC_DIR := src
BUILD_DIR := build
EXAMPLES_DIR := examples

LIB_SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
LIB_OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(LIB_SOURCES))
LIB_NAME := $(BUILD_DIR)/libmotionsensor.a

EXAMPLE_BIN := $(BUILD_DIR)/example_of_use

.PHONY: all clean

all: $(EXAMPLE_BIN)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(LIB_NAME): $(LIB_OBJECTS)
	ar rcs $@ $^

$(EXAMPLE_BIN): $(EXAMPLES_DIR)/example_of_use.cpp $(LIB_NAME)
	$(CXX) $(CXXFLAGS) $< -o $@ -L$(BUILD_DIR) -lmotionsensor $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
