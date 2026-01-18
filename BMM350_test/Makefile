# Makefile for BMM350 I2C example on Raspberry Pi

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99
LDFLAGS = -lm

# Common directory
COMMON_DIR = ../common

# Target executable
TARGET = bmm350_i2c

# Source files
SOURCES = bmm350_i2c.c $(COMMON_DIR)/common.c
OBJECTS = bmm350_i2c.o common.o

# Default target
all: $(TARGET)

# Build the executable
$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "Build complete: $(TARGET)"
	@echo "Run with: sudo ./$(TARGET)"

# Compile main source file
bmm350_i2c.o: bmm350_i2c.c $(COMMON_DIR)/common.h
	$(CC) $(CFLAGS) -c bmm350_i2c.c -o bmm350_i2c.o

# Compile common source file
common.o: $(COMMON_DIR)/common.c $(COMMON_DIR)/common.h
	$(CC) $(CFLAGS) -c $(COMMON_DIR)/common.c -o common.o

# Clean build artifacts
clean:
	rm -f $(OBJECTS) $(TARGET)
	@echo "Clean complete"

# Install (optional - requires root)
install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/
	@echo "Installed to /usr/local/bin/$(TARGET)"

# Uninstall
uninstall:
	rm -f /usr/local/bin/$(TARGET)
	@echo "Uninstalled $(TARGET)"

# Phony targets
.PHONY: all clean install uninstall
