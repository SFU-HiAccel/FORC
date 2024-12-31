# FORC

FORC is a high-throughput streaming-based FPGA accelerator overlay that supports processing ORC file format including ORC Zlib decompression, decoding and filtering, and its dataflow integration with Apache ORC. Experimental results show that FORC achieves up to 2.4GB/s throughput on AMD/Xilinx Alveo U280 FPGA, with a geomean speedup of 128x over the CPU. For more information please refer to our published paper.
[FPL 2024] FORC: A High-Throughput Streaming FPGA Accelerator for Optimized Row Columnar File Decoders in Big Data Engines[Link](https://www.sfu.ca/~zhenman/files/C38-FPL2024-FORC.pdf)

For decoding only check ORC_DECODER branch.

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
% cd orc_proc.xilinx_u280_xdma_201920_3.hw.xo.tapa/run-1/
% bash orc_proc.xilinx_u280_xdma_201920_3.hw_generate_bitstream.sh
% cd ../..
```

To build and run the Host Code run the following commands.

For Single Stripe Single Column to test with TAPA or run csim. CHECK "ORC FILTER" docx file for understanding the filter configurations.

```shell
% make Dgpp
% ./Dorc_final --bitstream xclbin/FORC.xclbin --comp <compressed-file-path> --orig <original-file-path> --is_orc=true --RR <right-range-of-filter> --VERIF=true
```

For Dataflow HW Runs. The code has been tested with g++ version of 7.5.0.

```shell
% make gpp
% ./orc_final --bitstream xclbin/FORC.xclbin --comp <compressed-file-path> --orig <original-file-path> --is_orc=true --RR <right-range-of-filter> --VERIF=true
```

# DESIGN LIMITATIONS

FORC currently supports the data decoding of DIRECT, DELTA, PATCHED BASE and SHORT REPEAT ORC encoders. It supports decoding of 8,16,24 and 32 bits encoded bit widths and runlengths of multiple of 64.
