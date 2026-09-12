.PHONY: all test sim clean

BUILD_DIR ?= build
CXX ?= g++

all:
	cmake -S . -B $(BUILD_DIR) -DCMAKE_CXX_COMPILER=$(CXX)
	cmake --build $(BUILD_DIR)

test: all
	ctest --test-dir $(BUILD_DIR) --output-on-failure

sim: all
	$(BUILD_DIR)/host/loco-sim --scenario all

clean:
	rm -rf $(BUILD_DIR)
