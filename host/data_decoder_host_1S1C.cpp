#include "data_decoder_host_1S1C.h"

void data_decoding(tapa::mmap<_512b> input_port, 
                    tapa::mmap<_512b> output_port0_32b_8b, 
                    tapa::mmap<_512b> output_port1_16b_8b,
                    tapa::mmap<_512b> output_port2_16b_8b,
                    tapa::mmap<_512b> output_port3_8b,
                    tapa::mmap<_512b> output_port4_Track,
                    uint32_t wait_count, 
                    uint32_t data_count
                    );

DEFINE_string(bitstream, "", "path to bitstream file, run csim if empty");


void async_readnorm(struct aiocb* aio_rf, void* data_in, int Fd, int vector_size_bytes, int offset)
{
    memset(data_in, 0, vector_size_bytes+PIPELINE_DEPTH);
    aio_rf->aio_buf = data_in;
    aio_rf->aio_fildes = Fd;
    aio_rf->aio_nbytes = vector_size_bytes;
    aio_rf->aio_offset = offset;
    int result = aio_read(aio_rf);
    if (result < 0)
    {
        printf("Read Failed: %d \n", result);
    }
}

int verif_all(int32_t *datain0, int32_t *datain1, int32_t *datain2, int32_t *datain3, int32_t *track);

void update_patch_data(int32_t *datain0, int32_t *datain1, int32_t *datain2, int32_t *datain3, int32_t *track);

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);

    uint32_t file_size_rem = 0;
    uint32_t KRNL_file_size_bytes = 0;
    uint32_t Data_offset = 0;
    uint32_t Data_length = 0;
    uint32_t KRNL_Data_Write = 0;
    uint32_t wait_count = 32;  //max FIFO depth for hardware and try 2147483 for csim
    std::string nvme_file;

    ////ORC READER////
    if(proc_ORC)
    {
        orc::ReaderOptions readerOpts;
        std::unique_ptr<orc::Reader> reader =
            orc::createReader(orc::readFile(orc_file, readerOpts.getReaderMetrics()), readerOpts);

        nvme_file = orc_file;
        std::cout << "{ \"name\": \"" << orc_file << "\",\n";
        uint64_t numberColumns = reader->getType().getMaximumColumnId() + 1;

        std::cout << "\n  \"file length\": " << reader->getFileLength() << ",\n";
        // std::cout << "  \"type\": \"" << reader->getType().toString() << "\",\n";
        
        nrows = reader->getNumberOfRows();
        std::cout << "  \"rows\": " << nrows << ",\n";
        uint64_t stripeCount = reader->getNumberOfStripes();
        std::cout << "  \"stripe count\": " << stripeCount << ",\n";
        
        if (stripeCount != 1) {
            std::cerr << "Error: Stripe count is not one. Current stripe count: " << stripeCount << std::endl;
            return 1;
        }

        for (uint64_t col = 0; col < numberColumns; ++col) {
        orc::ColumnEncodingKind encoding = reader->getStripe(0)->getColumnEncoding(col);
        std::cout << "         { \"column\": " << col << ", \"encoding\": \""
            << columnEncodingKindToString(encoding) << "\"";
        if (encoding == orc::ColumnEncodingKind_DICTIONARY ||
            encoding == orc::ColumnEncodingKind_DICTIONARY_V2) {
            std::cout << ", \"count\": " << reader->getStripe(0)->getDictionarySize(col);
        }
        std::cout << " }";
        std::cout << std::endl;
        }
        
        for (uint64_t str = 0; str < reader->getStripe(0)->getNumberOfStreams(); ++str) {
        if (str != 0) {
            std::cout << ",\n";
        }
        std::unique_ptr<orc::StreamInformation> stream = reader->getStripe(0)->getStreamInformation(str);
        std::cout << "        { \"id\": " << str << ", \"column\": " << stream->getColumnId()
            << ", \"kind\": \"" << streamKindToString(stream->getKind())
            << "\", \"offset\": " << stream->getOffset() << ", \"length\": " << stream->getLength()
            << " }";

            if(stream->getKind() == 1) 
            {
                std::cout << "\n \nData stream found" << std::endl;
                Data_offset = stream->getOffset();
                Data_length = stream->getLength();
            }
        }
        reader.reset(); //
    }
    else
    {

        nrows = Myrows;
        nvme_file = orc_file;
        std::ifstream input_file(orc_file, std::ios::binary);
        if (!input_file) {
            std::cerr << "Error: failed to open input file." << std::endl;
            return 1;
        }

        // Get the total file size
        input_file.seekg(0, std::ios::end);
        std::streampos file_size = input_file.tellg();
        input_file.close();

        Data_length = uint32_t(file_size);       
        Data_offset = 0;
        
    
    }
    std::cout << "Data_offset: " << Data_offset << std::endl;
    std::cout << "Data_length: " << Data_length << std::endl;

    file_size_rem = Data_length%64;
    if(file_size_rem!=0)
    {
        KRNL_file_size_bytes = Data_length + (64 - file_size_rem);
    }
    else
    {
        KRNL_file_size_bytes = Data_length;
    }
    KRNL_file_size_bytes = KRNL_file_size_bytes + PIPELINE_DEPTH;  //the pipeline depth of FPGA 64*8=512 + 64 = 576
    std::cout << "KRNL_file_size_bytes: " << KRNL_file_size_bytes << " bytes" << std::endl;
    std::cout << "Number of rows are: " << nrows << std::endl;
    

    ///////DECLARE READ WRITE HOST PTRs////////
    uint32_t remDiv = nrows%RSIZE_DIV;
    uint64_t dOut_Size_A = nrows / RSIZE_DIV;   //bcz of short repeat. else its nrows/16. One 512 can worst case contain 3 numbers.
    if(remDiv!=0)
    {
        dOut_Size_A = (dOut_Size_A + 1);
    }
    std::cout << "dOut_Size_A:  " << dOut_Size_A << std::endl;

    KRNL_file_size_bytes = KRNL_file_size_bytes/64; //Port Width is 512bits

    std::vector<_512b, tapa::aligned_allocator<_512b>> dataIn(KRNL_file_size_bytes);
    std::vector<_512b, tapa::aligned_allocator<_512b>> data_out(dOut_Size_A);   //contain SR as well. by default it should be dOut_Size_A/4
    std::vector<_512b, tapa::aligned_allocator<_512b>> data_out1(dOut_Size_A);   
    std::vector<_512b, tapa::aligned_allocator<_512b>> data_out2(dOut_Size_A);
    std::vector<_512b, tapa::aligned_allocator<_512b>> data_out3(dOut_Size_A);
    std::vector<_512b, tapa::aligned_allocator<_512b>> track_data(dOut_Size_A*2);   //Multiply by 4 bcz all info is there *2 bcz it has PLL info
    ///////////////////////////

    ///////Getting nvme ssd file desc////////
    auto FileTimeS = std::chrono::steady_clock::now();
    nvmeFd = open(nvme_file.c_str(), O_RDONLY); //O_SYNC O_DIRECT  O_RDONLY  O_RDWR
    if (nvmeFd < 0) {
        std::cerr << "ERROR: open " << nvme_file << "failed: " << std::endl;
        return EXIT_FAILURE;
    }
    auto FileTimeE = std::chrono::steady_clock::now();
    auto FileTime = std::chrono::duration_cast<std::chrono::microseconds>(FileTimeE - FileTimeS);
    std::cout << "File Opening Time (us):  " << FileTime.count() << std::endl;
    std::cout << "INFO: Successfully opened NVME SSD1 " << nvme_file << std::endl;
    ////////////DATA READ AND KERNEL CALL///////////////
    struct aiocb aio_rf;
    async_readnorm(&aio_rf, (void *)(dataIn.data()), nvmeFd, Data_length, Data_offset); 
    while( aio_error(&aio_rf) == EINPROGRESS ) {;}
    int aio_result = aio_error(&aio_rf);
    if (aio_result != 0) {
        perror("aio_error");
        std::cerr << "Asynchronous I/O error: " << strerror(aio_result) << std::endl;
        return 1;
    }
    int bytes_read = aio_return(&aio_rf);
    std::cout << "Bytes Read:" << bytes_read << std::endl;
    int64_t kernel_time_ns = 0;

    //max FIFO depth for hardware and try 2147483 for csim or larger
    if(FLAGS_bitstream != "")
    {
        wait_count = 32;
    }
    else
    {
        wait_count = WAIT_MAX;
    }

    kernel_time_ns += tapa::invoke(
        data_decoding, FLAGS_bitstream, 
        tapa::read_only_mmap<_512b>(dataIn),
        tapa::write_only_mmap<_512b>(data_out),
        tapa::write_only_mmap<_512b>(data_out1),
        tapa::write_only_mmap<_512b>(data_out2),
        tapa::write_only_mmap<_512b>(data_out3),
        tapa::write_only_mmap<_512b>(track_data),
        wait_count,
        KRNL_file_size_bytes
    );            
    std::cout << "Kernel Exec time(ms): " << (kernel_time_ns) * 1e-6 << std::endl;
    std::cout << "Data Input Size (MB): " << (float)(Data_length/(1024.0)/1024.0) << std::endl;
    std::cout << "Data Output Size (MB): " << (float)((nrows*4)/(1024.0*1024.0)) << std::endl;
    std::cout << "Kernel Input throughput(GB/s): " << (float)(Data_length)/(float)(kernel_time_ns) << std::endl;
    std::cout << "Kernel Output throughput(GB/s): " << (float)(nrows*4)/(float)(kernel_time_ns) << std::endl;
    
    
    int kernel_dout = 0;
    ap_uint<AXI_WIDTH> buf_out = 0;
    int32_t* data_OUT = reinterpret_cast<int32_t*>(aligned_alloc(4096, nrows*sizeof(int32_t)));
    //Data Verification
    #if 1
        update_patch_data(reinterpret_cast<int32_t*>(data_out.data()), 
                    reinterpret_cast<int32_t*>(data_out1.data()), 
                    reinterpret_cast<int32_t*>(data_out2.data()), 
                    reinterpret_cast<int32_t*>(data_out3.data()),
                    reinterpret_cast<int32_t*>(track_data.data())
                    );

        verif_all(reinterpret_cast<int32_t*>(data_out.data()), 
                    reinterpret_cast<int32_t*>(data_out1.data()), 
                    reinterpret_cast<int32_t*>(data_out2.data()), 
                    reinterpret_cast<int32_t*>(data_out3.data()),
                    reinterpret_cast<int32_t*>(track_data.data())
                    );
    #endif

    return 0;
}

int verif_all(int32_t *datain0, int32_t *datain1, int32_t *datain2, int32_t *datain3, int32_t *track)
{
    int64_t kernel_dout = 0;
    ap_int<AXI_WIDTH> buf_out0 = 0;
    ap_int<AXI_WIDTH> buf_out1 = 0;
    ap_int<AXI_WIDTH> buf_out2 = 0;
    ap_int<AXI_WIDTH> buf_out3 = 0;

    ap_int<AXI_WIDTH> *Data_Out0;
    Data_Out0 = (ap_int<AXI_WIDTH>*)(datain0);

    ap_int<AXI_WIDTH> *Data_Out1;
    Data_Out1 = (ap_int<AXI_WIDTH>*)(datain1);

    ap_int<AXI_WIDTH> *Data_Out2;
    Data_Out2 = (ap_int<AXI_WIDTH>*)(datain2);

    ap_int<AXI_WIDTH> *Data_Out3;
    Data_Out3 = (ap_int<AXI_WIDTH>*)(datain3);

    _128b *mTr;
    mTr = (_128b*)(track);

    std::ifstream in_file(check_file);

    if (!in_file) {
        std::cerr << "Error: failed to open input file." << std::endl;
        return -1;
    }

    std::string line;
    int j = 0;
    int n = 0;
    int d0_iter = 0;
    int d1_iter = 0;
    int d2_iter = 0;
    int d3_iter = 0;
    uint32_t tRun = 0;

    uint32_t batch = 0;

    uint16_t decType = 0;
    uint16_t runLength = 0;
    uint8_t PLL = 0;
    int64_t number = 0;

    while(n < nrows)
    {
        j = 0;
        batch = 0;

        PLL = mTr->range(7,0);      //8
        decType = mTr->range(71,64);  //8
        runLength = mTr->range(95,80);    //16
        // mTr++;

        if(runLength != 0)
        {
            if(PLL!=0)
            {
                mTr++;
            }
            else
            {
                if(runLength <= 64)
                {
                    tRun = runLength;
                }
                else
                {
                    tRun = 64;
                }

                while ((batch < tRun) && (n < nrows))
                {
                    std::getline(in_file, line);
                    if(j == 0)
                    {
                        buf_out0 = Data_Out0[d0_iter];       //Data_Out_Check
                        buf_out1 = Data_Out1[d1_iter];       //Data_Out_Check
                        buf_out2 = Data_Out2[d2_iter];       //Data_Out_Check
                        buf_out3 = Data_Out3[d3_iter];       //Data_Out_Check

                        d0_iter++;
                        d1_iter++;
                        d2_iter++;
                        d3_iter++;
                        mTr++;

                        kernel_dout = buf_out0.range(31,0);
                        buf_out0 = buf_out0 >> 32;
                        ++j;

                    }
                    else
                    {
                        if (j > 47) {
                            kernel_dout = buf_out3.range(31, 0);
                            buf_out3 = buf_out3 >> 32;
                        } else if (j > 31) {
                            kernel_dout = buf_out2.range(31, 0);
                            buf_out2 = buf_out2 >> 32;
                        }
                        else if (j > 15) {
                            kernel_dout = buf_out1.range(31, 0);
                            buf_out1 = buf_out1 >> 32;
                        }
                        else {
                            kernel_dout = buf_out0.range(31, 0);
                            buf_out0 = buf_out0 >> 32;
                        }
                        j+=1;
                    }

                    try {
                        
                        number = std::stoll(line);
                        n++;
                        if(number != kernel_dout)
                        {
                            std::cout << "number mismatch at: " << n << " ; Actual Data: " << number << " ; Kernel Data: " << kernel_dout << std::endl;
                            std::cout << "DEBUG J_val: " << j << std::endl;
                            std::cout << "Total Numbers are: " << nrows << std::endl;
                            in_file.close();
                            return -1;
                        }
                        // std::cout << number << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "Error: Invalid number format in line: " << line << std::endl;
                    }
                    ++batch;

                    

                }
                
            }
        }
        
    }

    if(n >= nrows)
    {
        std::cout << "Passed, Total Numbers matched: " << n << std::endl;
    }
    else 
    {
        std::cout << "Failed, Total Numbers matched: " << n << std::endl;
    }

    in_file.close();
    return 1;
}


void update_patch_data(int32_t *datain0, int32_t *datain1, int32_t *datain2, int32_t *datain3, int32_t *track)
{

    _128b *mTr;
    mTr = (_128b*)(track);

    uint16_t decType = 0;
    uint16_t runLength = 0;
    int32_t BV = 0;
    uint8_t PLL = 0;
    uint16_t gap_val = 0;
    int32_t patch_val = 0;

    int32_t totalCount = nrows;

    bool latchRL = 1;
    bool latchPA = 1;
    uint16_t TRL = 0;
    uint32_t data_count = 0;
    uint32_t crow = 0;
    uint32_t patch_Didx = 0;
    uint16_t offset_gap = 0;
    uint16_t gap_mod = 0;
    uint16_t prev_gap = 0;
    uint16_t dataPtr = 0;
    uint16_t subVal = 0;
    uint32_t fdiv = 0;
    uint16_t offset = 0;
    uint32_t pll_count = 0;
    uint32_t Data_idx = 0;

    while(crow < totalCount)
    {
        
        PLL = mTr->range(7,0);      //8
        gap_val = mTr->range(23,8);   //16
        patch_val = mTr->range(55,24);         //32
        decType = mTr->range(71,64);  //16  (79,64)
        runLength = mTr->range(95,80);    //16
        BV = mTr->range(127,96);       //32

        mTr++;
        if(latchRL)
        {
            latchRL = 0;
            TRL = runLength;
        }


        if(decType != PATCHED)
        {
            //Skip
            data_count += 16;
            latchPA = 1;
            //loop control
            if(TRL <= 64)
            {
                latchRL = 1;
                crow += TRL;
                TRL = 0;
            }
            else
            {
                TRL -= 64;
                crow += 64;
            }
            
        }
        else
        {
            //fix for PLL count to relatch the RL and PA latch
            //get the start idx from where the patched data is starting, only once
            if(latchPA)
            {
                latchPA = 0;
                patch_Didx = data_count; //not subtracting bcz added afterwards
            }
            //Process
            if(PLL!=0)  //reading the Patch Gap values
            {
                pll_count += 1;
                if(pll_count == PLL)
                {
                    crow += TRL;    //add in the end bcz of last data
                    pll_count = 0;
                    latchRL = 1;
                    latchPA = 1;
                }
                //get data index from gap and add the patch value
                if(gap_val == 255 and patch_val == 0)
                {
                    //skip to next, no data updates
                    offset_gap = gap_val;
                }
                else    //update the data
                {
                    gap_val +=  prev_gap;
                    gap_val += offset_gap;
                    offset_gap = 0;
                    prev_gap = gap_val;

                    gap_mod = gap_val%64;   //batch repeat after 64
                    dataPtr = gap_mod/16;   //pointer repeats after 16

                    fdiv = gap_val/64;
                    subVal = (fdiv*64)+(dataPtr*16);
                    offset = gap_val-subVal;

                    Data_idx = (fdiv*16) + patch_Didx + offset;

                    switch (dataPtr)
                    {
                        case 0:
                            datain0[Data_idx] += patch_val;
                            break;
                        case 1:
                            datain1[Data_idx] += patch_val;
                            break;
                        case 2:
                            datain2[Data_idx] += patch_val;
                            break;
                        case 3:
                            datain3[Data_idx] += patch_val;
                            break;
                        
                        default:
                            break;
                    }
                }

            }
            else        //keep reading metadata till you find PLL
            {
                //skip
                data_count += 16;
                prev_gap = 0;
                //loop control
                if(TRL <= 64)
                {
                    TRL = 0;
                }
                else
                {
                    TRL -= 64;
                    crow += 64;
                }
                
            }
        }
    }

}