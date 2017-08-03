# Makefile for cp2110-c example programs

    
CC       = gcc
CFLAGS   = -Wall -g -fpic

CXX      ?= g++
CXXFLAGS ?= -Wall -g -fpic

LDFLAGS  ?= -Wall -g

INCLUDES = -I../hidapi/hidapi -I../include
LIBS_USB  = `pkg-config libusb-1.0 --libs` -lrt -lpthread
CP2110   = ../src/cp2110.c
BIN_DIR = bin

all: test lib
lib: cp2110-c.so

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

cp2110.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $(CP2110) -o cp2110.o


test: test.o cp2110.o ../hidapi/libusb/hid.o
	$(CC) $(CFLAGS) $(INCLUDES) $^  $(LIBS_USB) -o $(BIN_DIR)/test
    
# Shared Libs
cp2110-c.so: cp2110.o ../hidapi/libusb/hid.o
	$(CC) $(LDFLAGS) -shared -fpic -Wl,-soname,$@.0 $^ $(LIBS_USB) -o $(BIN_DIR)/$@

clean:
	rm -f *.o bin/test bin/cpp2110-c.so