# Build on the Pi:  make        -> build/pothole        make probe -> build/hailo_probe
#                   make sim7600 -> build/sim7600_test (4G HAT / GPS bench checks)
# Dependencies:     sudo apt install build-essential pkg-config libcamera-dev libturbojpeg0-dev libcurl4-openssl-dev dkms hailo-all

CC  ?= gcc
CXX ?= g++

CFLAGS   ?= -O2 -g
CXXFLAGS ?= -O2 -g
CFLAGS   += -std=c11 -D_GNU_SOURCE -Wall -Wextra -pthread $(shell pkg-config --cflags libturbojpeg libcurl)
# C++20: current libcamera headers use std::span
CXXFLAGS += -std=c++20 -Wall -Wextra -pthread $(shell pkg-config --cflags libcamera)

# HailoRT (from `sudo apt install hailo-all`) ships only the versioned library,
# libhailort.so.4.23.0, with no libhailort.so for -lhailort, so link it by path.
HAILORT_LIB := $(firstword $(wildcard /usr/lib/libhailort.so /usr/lib/libhailort.so.* \
                                      /usr/lib/aarch64-linux-gnu/libhailort.so*))

LDLIBS += $(shell pkg-config --libs libcamera libturbojpeg libcurl) $(HAILORT_LIB) -lm -pthread

BUILD := build
BIN   := $(BUILD)/pothole
DETECT_OBJS := $(BUILD)/hailo_model.o $(BUILD)/yolo_seg.o $(BUILD)/image.o
OBJS  := $(BUILD)/main.o $(BUILD)/stream.o $(BUILD)/jpeg.o $(BUILD)/camera.o $(BUILD)/overlay.o \
         $(BUILD)/sysstats.o $(BUILD)/runlog.o $(BUILD)/geometry.o $(BUILD)/tracker.o \
         $(BUILD)/potholes.o $(BUILD)/recorder.o $(BUILD)/calibrate.o $(BUILD)/gps.o $(BUILD)/serial.o \
         $(BUILD)/uploader.o $(DETECT_OBJS)

REPLAY      := $(BUILD)/pothole_replay
REPLAY_OBJS := $(BUILD)/replay_video.o $(BUILD)/potholes.o $(BUILD)/recorder.o $(BUILD)/geometry.o \
               $(BUILD)/tracker.o $(BUILD)/gps.o $(BUILD)/serial.o $(DETECT_OBJS)

SIM7600      := $(BUILD)/sim7600_test
SIM7600_OBJS := $(BUILD)/sim7600_test.o $(BUILD)/gps.o $(BUILD)/serial.o $(BUILD)/uploader.o

PROBE      := $(BUILD)/hailo_probe
PROBE_OBJS := $(BUILD)/hailo_probe.o $(DETECT_OBJS)

all: $(BIN) $(SIM7600)

# Linked with the C++ compiler because camera.cpp pulls in libstdc++.
$(BIN): $(OBJS) | hailort
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

probe: $(PROBE)

# SIM7600X HAT checks: ports, signal, GPS, data link, one upload. No camera or Hailo needed.
sim7600: $(SIM7600)

$(SIM7600): $(SIM7600_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS) $(shell pkg-config --libs libcurl) -lm -pthread

# Runs a video file through the pipeline (needs ffmpeg) to compare with the Python results.
replay: $(REPLAY)

$(REPLAY): $(REPLAY_OBJS) | hailort
	$(CC) -o $@ $^ $(LDFLAGS) $(HAILORT_LIB) $(shell pkg-config --libs libturbojpeg) -lm -pthread

$(PROBE): $(PROBE_OBJS) | hailort
	$(CC) -o $@ $^ $(LDFLAGS) $(HAILORT_LIB) $(shell pkg-config --libs libturbojpeg) -lm -pthread

hailort:
	@test -n "$(HAILORT_LIB)" || { echo "libhailort not found: sudo apt install dkms hailo-all && sudo reboot"; exit 1; }

$(BUILD)/%.o: src/%.c src/*.h | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/%.o: src/%.cpp src/*.h | $(BUILD)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)

.PHONY: all probe replay sim7600 clean hailort
