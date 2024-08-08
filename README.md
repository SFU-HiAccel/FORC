# FORC

FORC is a high-throughput streaming-based FPGA accelerator overlay that supports different ORC file format decoders, and its dataflow integration with Apache ORC. Experimental results show that FORC achieves up to 12.9GB/s decoding throughput on AMD/Xilinx Alveo U280 FPGA, with a geomean speedup of 65x (up to 335x) over the CPU. For more information please refer to our published paper.
[FPL 2024] FORC: A High-Throughput Streaming FPGA Accelerator for Optimized Row Columnar File Decoders in Big Data Engines[Link](https://www.sfu.ca/~zhenman/files/C38-FPL2024-FORC.pdf)

# SYSTEM REQUIREMENTS

FORC has been tested on Xilinx AMD Alveo U280 FPGA, built using [TAPA](https://github.com/UCLA-VAST/tapa), version 0.0.20221113.1. See [here](https://tapa.readthedocs.io/en/release/installation.html) for installation instructions.

TAPA uses AMD/Xilinx Vitis and Vivado for compilation and builds. FORC has been tested to work with Vitis 2021.2.

To support the ORC integration. Built the C++ ORC library using the instruction [here](https://github.com/apache/orc).
NOTE: For older Ubuntu version you can see the work around [here](https://github.com/apache/orc/issues/1613).

Once ORC is built it generates a "*.tar.gz" packaged file. You need to extract it and "cd" into that folder.
Then run the following command:

```shell
% tar -xvf *.tar.gz
% cd (Extracted_Folder_Name)
% export ORC_PATH=$(pwd)
```

# USAGE

The FORC design contains two host codes. 
    - One for single stripe and single column testing for both hardware and software runs. 
    - Second for dataflow implementation only for hardware runs.

To build the RTL design run the following commands.

```shell
% make rtl_gen
% cd decoder.xilinx_u280_xdma_201920_3.hw.xo.tapa/run-1/
% bash decoder.xilinx_u280_xdma_201920_3.hw_generate_bitstream.sh
% cd ../..
```

To build and run the Host Code run the following commands.

For Single Stripe Single Column

```shell
% make Sgpp
% ./decoder1S1C --bitstream decoder.xilinx_u280_xdma_201920_3.hw.xo.tapa/run-1/vitis_run_hw/data_decoding_xilinx_u280_xdma_201920_3.xclbin
```

For Dataflow HW Runs

```shell
% make gpp
% ./decoder --bitstream decoder.xilinx_u280_xdma_201920_3.hw.xo.tapa/run-1/vitis_run_hw/data_decoding_xilinx_u280_xdma_201920_3.xclbin
```

# DESIGN LIMITATIONS

FORC currently supports the data decoding of DIRECT, DELTA, PATCHED BASE and SHORT REPEAT ORC encoders. It supports decoding of 8,16,24 and 32 bits encoded bit widths and runlengths of multiple of 64.
