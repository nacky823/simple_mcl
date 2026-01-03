# Makefile (CMake wrapper)
BUILD_DIR := build
TARGET := simple_mcl

.PHONY: all configure build run clean distclean rebuild

all: build

configure:
	@cmake -S . -B $(BUILD_DIR)

build: configure
	@cmake --build $(BUILD_DIR) -j

run: build
	@./$(BUILD_DIR)/$(TARGET)

clean:
	@cmake --build $(BUILD_DIR) --target clean 2>/dev/null || true

distclean:
	@rm -rf $(BUILD_DIR)

rebuild: distclean build
