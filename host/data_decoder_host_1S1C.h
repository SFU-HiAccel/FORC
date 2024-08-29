#ifndef data_decoder_host_1S1C_h
#define data_decoder_host_1S1C_h

#include <iostream>
#include <fstream>
#include <vector>
#include <bitset>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <stdio.h>
#include <aio.h>
#include <fcntl.h>
#include <chrono>

#include <orc/orc-config.hh>
#include <orc/Reader.hh>
#include <orc/Exceptions.hh>
#include <orc/OrcFile.hh>

#include <tapa.h>
//leave this space between tapa.h and ap_int.h
#include <ap_int.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

// #define __CL_ENABLE_EXCEPTIONS
// #include <CL/cl.h>

#include <tinyxml.h>
#include <xclbin.h>
#define CL_HPP_ENABLE_EXCEPTIONS
// #define CL_HPP_TARGET_OPENCL_VERSION 200
#include <CL/cl2.hpp>

#include <CL/opencl.h>
// #include <CL/cl_ext_xilinx.h>

#define CL_DEVICE_PCIE_BDF              0x1120  // BUS/DEVICE/FUNCTION
#include "opencl_util.h"

extern "C" {
    int aio_write(struct aiocb*);
    int aio_read(struct aiocb*);
    int aio_error(const struct aiocb *aiocbp);
    ssize_t aio_return(struct aiocb *aiocbp);
    int aio_suspend(const struct aiocb * const cblist[], int n, const struct timespec *timeout);
}

#define WAIT_MAX 2147483

int nvmeFd = -1;
const uint32_t AXI_WIDTH = 512;
const uint16_t AXI_WIDTH_HH = 128;
typedef ap_uint<AXI_WIDTH> _512b;
typedef ap_int<AXI_WIDTH> _512bi;
typedef ap_uint<AXI_WIDTH_HH> _128b;
typedef ap_uint<32> _32b;

const uint32_t RSIZE_DIV = 16;   //for SR it should be 4 else 16
const uint32_t PIPELINE_DEPTH = 576;
uint32_t nrows = 0;
uint32_t Myrows = 4782212;      //if proc_ORC is 0 you need to provide the rows.
bool proc_ORC = 0;
std::string orc_file = "/localhdd/awa159/orc_dataset/orc_decData/lineitem_col1_stripe10_data0.bin";     //"test_data/8_bit.orc";           // /localhdd/awa159/orc_dataset/orc_decData/lineitem_col1_stripe10_data0.bin
std::string check_file = "/localhdd/awa159/orc_dataset/orc_decData/lineitem_col1_lastStripe.bin";   //"test_data/8_bit_data.bin";    // /localhdd/awa159/orc_dataset/orc_decData/lineitem_col1_lastStripe.bin

const uint8_t SR = 0;
const uint8_t DIRECT = 1;
const uint8_t PATCHED = 2;
const uint8_t DELTA = 3;

#endif // data_decoder_host_1S1C_h
