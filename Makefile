# Makefile

in1 := "input_port"
out1 := "output_port0_32b_8b"
out2 := "output_port1_16b_8b"
out3 := "output_port2_16b_8b"
out4 := "output_port3_8b"
out5 := "output_port4_Track"
out6 := "FilterConf_port"
out7 := "data_Idx"

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
OUTPUT := orc_final
DOUTPUT := Dorc_final
TARGET ?= hw
ifeq ($(filter $(TARGET),sw_emu hw_emu hw),)
$(error TARGET is not sw_emu, hw_emu or hw)
endif

TOP := orc_proc

SRC_NAME := orc_proc
SOURCES := kernel/$(SRC_NAME).cpp host/$(SRC_NAME)_host.cpp
DSOURCES := kernel/$(SRC_NAME).cpp host/$(SRC_NAME)_host_1S1C.cpp


all: $(OUTPUT) rtl_gen

$(OUTPUT): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $^ -I$(VITIS_HLS_INCLUDE) -I$(XILINX_XRT_INCLUDE) -L$(XILINX_XRT_LIB) $(ORC_INCLUDE) $(ORC_LIB) $(LIBRARIES) $(ORC_LIBRARIES) -DHAVE_CL2 -DCL_HPP_TARGET_OPENCL_VERSION=120 -DCL_HPP_MINIMUM_OPENCL_VERSION=120

gpp: clean_gpp $(OUTPUT)

$(DOUTPUT): $(DSOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $^ -I$(VITIS_HLS_INCLUDE) -I$(XILINX_XRT_INCLUDE) -L$(XILINX_XRT_LIB) $(ORC_INCLUDE) $(ORC_LIB) $(LIBRARIES) $(ORC_LIBRARIES) -DHAVE_CL2 -DCL_HPP_TARGET_OPENCL_VERSION=120 -DCL_HPP_MINIMUM_OPENCL_VERSION=120

Dgpp: clean_Dgpp $(DOUTPUT)

# "min_area_limit": 0.65,
# "max_area_limit": 0.85,
# "min_slr_width_limit": 10000,
# "max_slr_width_limit": 15000,
# "max_search_time": 600,
# "floorplan_strategy": "HALF_SLR_LEVEL_FLOORPLANNING",
# "floorplan_opt_priority": "AREA_PRIORITIZED",
# "enable_hbm_binding_adjustment": false

# --min-area-limit 0.40 \
# --max-area-limit 0.60 \
# --min-slr-width-limit 8000 \
# --max-slr-width-limit 10000 \
# --floorplan-strategy "SLR_LEVEL_FLOORPLANNING" \
# --floorplan-opt-priority "SLR_CROSSING_PRIORITIZED" \
# --max-search-time 60000 \

# --enable-synth-util \
# --run-floorplan-dse \
# --enable-hbm-binding-adjustment \
# --clock-period 2.22 \
# get the platform from user input 

# --enable-synth-util \
# --min-area-limit 0.35 \
# --max-area-limit 0.45

CLK ?= 3.333

rtl_gen:
	platform=$(PLATFORM)  
	tapac -o $(OUTPUT).$(PLATFORM).hw.xo kernel/$(SRC_NAME).cpp \
		--platform $(PLATFORM) \
		--clock-period $(CLK) \
		--top $(TOP) \
		--work-dir $(OUTPUT).$(PLATFORM).hw.xo.tapa \
		--connectivity kernel/$(INI) \
		--enable-floorplan \
		--floorplan-output constraint.tcl \
		$(EXTRA_FLAGS) \
		--enable-synth-util \
		--max-parallel-synth-jobs 12 \
		--enable-hbm-binding-adjustment \
		--run-tapacc \
		--run-hls \
		--generate-task-rtl \
		--run-floorplanning \
		--run-floorplan-dse \
		--generate-top-rtl \
		--pack-xo

build_hw: update_route append_resource_gen
	cd $(OUTPUT).$(PLATFORM).hw.xo.tapa/run-1 && \
	sed -i 's/^# TARGET=hw/TARGET=hw/' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh && \
	sed -i 's/^TARGET=hw_emu/# TARGET=hw_emu/' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh && \
	sed -i 's/^DEBUG=-g/# DEBUG=-g/' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh && \
	bash $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh

build_cosim: append_resource_gen
	cd $(OUTPUT).$(PLATFORM).hw.xo.tapa/run-1 && \
	sed -i 's/^TARGET=hw/# TARGET=hw/' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh && \
	sed -i 's/^# TARGET=hw_emu/TARGET=hw_emu/' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh && \
	sed -i 's/^# DEBUG=-g/DEBUG=-g/' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh && \
	bash $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh

update_route:
	cd $(OUTPUT).$(PLATFORM).hw.xo.tapa/run-1 && \
	if ! grep -q '^ROUTE_STRATEGY=' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh; then \
	  sed -i '/^PLACEMENT_STRATEGY=/a ROUTE_STRATEGY="AggressiveExplore"' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh; \
	fi; \
	if ! grep -q '^\s*--vivado\.prop=run\.impl_1\.STEPS\.ROUTE_DESIGN\.ARGS\.DIRECTIVE=\$$ROUTE_STRATEGY \\' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh; then \
	  sed -i 's/^\(\s*\)--vivado\.prop=run\.impl_1\.STEPS\.ROUTE_DESIGN\.ARGS\.DIRECTIVE=.*/\1--vivado.prop=run.impl_1.STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE=$$ROUTE_STRATEGY \\/' $(OUTPUT).$(PLATFORM).hw_generate_bitstream.sh; \
	fi

# to view accelerator resource
append_resource_gen:
	cd $(OUTPUT).$(PLATFORM).hw.xo.tapa/run-1 && \
	if ! grep -q "report_accelerator_utilization -kernels { pfm_top_wrapper:pfm_top_i/dynamic_region/$(TOP):$(TOP) }" constraint.tcl; then \
		echo "" >> constraint.tcl && \
		echo "report_accelerator_utilization -kernels { pfm_top_wrapper:pfm_top_i/dynamic_region/$(TOP):$(TOP) } -file $(PWD)/$(OUTPUT).$(PLATFORM).hw.xo.tapa/run-1/vitis_run_hw/kernel_util_Full_Routed.rpt -name kernel_util_Full_Routed -json" >> constraint.tcl; \
	fi; \
	if ! grep -q "report_utilization -hierarchical -file $(PWD)/$(OUTPUT).$(PLATFORM).hw.xo.tapa/run-1/vitis_run_hw/rep_hierarchical.rpt -hierarchical_depth 6" constraint.tcl; then \
		echo "" >> constraint.tcl && \
		echo "report_utilization -hierarchical -file $(PWD)/$(OUTPUT).$(PLATFORM).hw.xo.tapa/run-1/vitis_run_hw/rep_hierarchical.rpt -hierarchical_depth 6" >> constraint.tcl; \
	fi

# to run with manual floorplan
# Mrtl_gen:
# 	platform=$(PLATFORM)  
# 	tapac -o $(OUTPUT).$(PLATFORM).hw.xo $(SRC_NAME).cpp \
# 		--platform $(PLATFORM) \
# 		--clock-period 3.33 \
# 		--top $(TOP) \
# 		--work-dir $(OUTPUT).$(PLATFORM).hw.xo.tapa \
# 		--connectivity $(INI) \
# 		--enable-floorplan \
# 		--floorplan-output constraint.tcl \
# 		$(EXTRA_FLAGS) \
# 		--enable-synth-util \
# 		--enable-hbm-binding-adjustment \
# 		--max-parallel-synth-jobs 24 \
# 		--run-tapacc \
# 		--run-hls \
# 		--generate-task-rtl \
# 		--floorplan-pre-assignments floorplan-region-to-instances16x.json \
# 		--run-floorplanning \
# 		--run-floorplan-dse \
# 		--generate-top-rtl \
# 		--pack-xo

clean_rtl:
	rm -rf $(OUTPUT).*.hw.* $(OUTPUT).*.hw.*.tapa
	rm -rf $(OUTPUT).$(PLATFORM).*
	rm -rf _x
	rm *.log

clean_gpp:
	rm -rf $(OUTPUT)

clean_Dgpp:
	rm -rf $(DOUTPUT)

clean: clean_gpp clean_rtl

