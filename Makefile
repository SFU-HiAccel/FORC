# Makefile

in1 := "input_port"
out1 := "output_port0_32b_8b"
out2 := "output_port1_16b_8b"
out3 := "output_port2_16b_8b"
out4 := "output_port3_8b"
out5 := "output_port4_Track"

MK_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
CUR_DIR := $(patsubst %/,%,$(dir $(MK_PATH)))
CUR_DIR ?= $(CUR_DIR)

#default 
PLATFORM ?= xilinx_u280_xdma_201920_3

# Compiler
CXX := g++

# Compiler flags
CXXFLAGS := -O2
server := $(shell hostname)

EXTRA_FLAGS := --read-only-args $(in1) 
EXTRA_FLAGS += --write-only-args $(out1) --write-only-args $(out2) --write-only-args $(out3) --write-only-args $(out4)
EXTRA_FLAGS += --write-only-args $(out5)
INI := link_hbm.ini

VITIS_HLS_INCLUDE := $(XILINX_HLS)/include/
VITIS_HLS_LIB := $(XILINX_HLS)/lib/
XILINX_XRT_LIB := $(XILINX_XRT)/lib/
XILINX_XRT_INCLUDE := $(XILINX_XRT)/include/

ORC_INCLUDE := -I $(ORC_PATH)/include
ORC_LIB := -L $(ORC_PATH)/lib

LIBRARIES := -std=c++17 -ltinyxml -lpthread -lrt -lgmp -lmpfr -ltapa -lfrt -lglog -lgflags -lOpenCL
ORC_LIBRARIES := -lorc -lprotobuf -lzstd -lsnappy -llz4 -lpthread -lz

# Initializers
OUTPUT := decoder2
OUTPUT1S1C := decoder1S1C
TARGET ?= hw
ifeq ($(filter $(TARGET),sw_emu hw_emu hw),)
$(error TARGET is not sw_emu, hw_emu or hw)
endif

TOP := data_decoding

SRC_NAME := data_decoder
SOURCES := kernel/$(SRC_NAME).cpp host/$(SRC_NAME)_host.cpp

all: $(OUTPUT) rtl_gen

$(OUTPUT): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $^ -I$(VITIS_HLS_INCLUDE) -I$(XILINX_XRT_INCLUDE) -L$(XILINX_XRT_LIB) $(ORC_INCLUDE) $(ORC_LIB) $(LIBRARIES) $(ORC_LIBRARIES) -DHAVE_CL2 -DCL_HPP_TARGET_OPENCL_VERSION=120 -DCL_HPP_MINIMUM_OPENCL_VERSION=120

gpp: clean_gpp $(OUTPUT)

SOURCES1S1C := kernel/$(SRC_NAME).cpp host/$(SRC_NAME)_host_1S1C.cpp
$(OUTPUT1S1C): $(SOURCES1S1C)
	$(CXX) $(CXXFLAGS) -o $@ $^ -I$(VITIS_HLS_INCLUDE) -I$(XILINX_XRT_INCLUDE) -L$(XILINX_XRT_LIB) $(ORC_INCLUDE) $(ORC_LIB) $(LIBRARIES) $(ORC_LIBRARIES) -DHAVE_CL2 -DCL_HPP_TARGET_OPENCL_VERSION=120 -DCL_HPP_MINIMUM_OPENCL_VERSION=120

Sgpp: clean_gpp $(OUTPUT1S1C)

rtl_gen:
	platform=$(PLATFORM)  
	tapac -o $(OUTPUT).$(PLATFORM).hw.xo kernel/$(SRC_NAME).cpp \
		--platform $(PLATFORM) \
		--clock-period 3.33 \
		--top $(TOP) \
		--work-dir $(OUTPUT).$(PLATFORM).hw.xo.tapa \
		--connectivity kernel/$(INI) \
		--enable-floorplan \
		--floorplan-output constraint.tcl \
		$(EXTRA_FLAGS) \
		--enable-synth-util \
		--max-parallel-synth-jobs 24 \
		--run-tapacc \
		--run-hls \
		--generate-task-rtl \
		--run-floorplanning \
		--enable-hbm-binding-adjustment \
		--run-floorplan-dse \
		--generate-top-rtl \
		--pack-xo

clean_rtl:
	rm -rf $(OUTPUT).*.hw.* $(OUTPUT).*.hw.*.tapa
	rm -rf $(OUTPUT).$(PLATFORM).*
	rm -rf _x
	rm *.log

clean_gpp:
	rm -rf $(OUTPUT) $(OUTPUT1S1C)

clean: clean_gpp clean_rtl

