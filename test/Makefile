# Host test of the real Rampa_MCU.ino (Arduino API stubbed in stubs/).
CXX ?= g++
OUT ?= build
all: rampa
rampa:
	mkdir -p $(OUT)
	$(CXX) -std=c++17 -O1 -Wall -Wno-unused-function -Wno-unused-variable -Istubs -I../Rampa_MCU -x c++ rampa_host_test.cpp -o $(OUT)/rampa_test && $(OUT)/rampa_test
.PHONY: all rampa
