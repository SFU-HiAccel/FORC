#include "data_decoder_host.h"

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

void writeZeros(void* ptr, uint32_t offset, uint32_t size) {
    // Calculate the address with the offset
    void* target = static_cast<char*>(ptr) + offset;

    // Write zeros to the memory region
    memset(target, 0, size);
}

int verif_all(int32_t *datain0, int32_t *datain1, int32_t *datain2, int32_t *datain3, int32_t *track);

void update_patch_data(int32_t *datain0, int32_t *datain1, int32_t *datain2, int32_t *datain3, int32_t *track);

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);

    uint32_t file_size_rem = 0;
    uint32_t KRNL_file_size_bytes = 0;
    std::vector<uint32_t> Data_offsets;
    std::vector<uint32_t> Data_lengths;
    std::vector<uint64_t> stripe_rows;  // Array to store the number of rows in each stripe
    uint32_t Data_offset = 0;
    uint32_t KRNL_Data_Write = 0;
    uint32_t wait_count = 32;  //max FIFO depth for hardware and try 2147483 for csim
    std::string nvme_file;

    uint32_t max_stripe_rows = 0;  // Variable to store the maximum number of rows in any stripe
    uint32_t max_data_length = 0;  // Variable to store the maximum data length
    uint32_t total_data_length = 0;  // Variable to store the sum of all data lengths

    ////ORC READER////
    orc::ReaderOptions readerOpts;
    std::unique_ptr<orc::Reader> reader =
        orc::createReader(orc::readFile(orc_file, readerOpts.getReaderMetrics()), readerOpts);

    nvme_file = orc_file;
    
    uint64_t numberColumns = reader->getType().getMaximumColumnId() + 1;
    uint64_t nrows = reader->getNumberOfRows();
    uint64_t stripeCount = reader->getNumberOfStripes();

    #ifdef PRINT_DEBUG
        std::cout << "{ \"name\": \"" << orc_file << "\",\n";
        std::cout << "\n  \"file length\": " << reader->getFileLength() << ",\n";
        std::cout << "  \"rows\": " << nrows << ",\n";
        std::cout << "  \"stripe count\": " << stripeCount << ",\n";
    #endif

    if (stripeCount == 0) {
        std::cerr << "Error: Stripe count is zero. Read stripe count: " << stripeCount << std::endl;
        return 1;
    }

    for (uint64_t col = 0; col < numberColumns; ++col) {
        orc::ColumnEncodingKind encoding = reader->getStripe(0)->getColumnEncoding(col);
        #ifdef PRINT_DEBUG
            std::cout << "         { \"column\": " << col << ", \"encoding\": \""
                    << columnEncodingKindToString(encoding) << "\"";
            if (encoding == orc::ColumnEncodingKind_DICTIONARY ||
                encoding == orc::ColumnEncodingKind_DICTIONARY_V2) {
                std::cout << ", \"count\": " << reader->getStripe(0)->getDictionarySize(col);
            }
            std::cout << " }";
            std::cout << std::endl;
        #endif
    }

    for (uint64_t str = 0; str < stripeCount; ++str) {
        auto stripe = reader->getStripe(str);
        stripe_rows.push_back(stripe->getNumberOfRows());  // Populate the stripe_rows array
        // std::cout << "Stripe " << str << " has " << stripe->getNumberOfRows() << " rows.\n";  // Print the number of rows in each stripe

        for (uint64_t streamIdx = 0; streamIdx < stripe->getNumberOfStreams(); ++streamIdx) {
            std::unique_ptr<orc::StreamInformation> stream = stripe->getStreamInformation(streamIdx);
            #ifdef PRINT_DEBUG
                if (streamIdx != 0) {
                    std::cout << ",\n";
                }
                std::cout << "        { \"id\": " << streamIdx << ", \"column\": " << stream->getColumnId()
                        << ", \"kind\": \"" << streamKindToString(stream->getKind())
                        << "\", \"offset\": " << stream->getOffset() << ", \"length\": " << stream->getLength()
                        << " }";
            #endif
            if (stream->getKind() == 1) {
                // std::cout << "Data stream found" << std::endl;
                Data_offsets.push_back(stream->getOffset());
                Data_lengths.push_back(stream->getLength());
            }
        }
    }
    // Finding the maximum values
    if (!stripe_rows.empty()) {
        max_stripe_rows = static_cast<uint32_t>(*std::max_element(stripe_rows.begin(), stripe_rows.end()));
    } else {
        std::cout << "stripe_rows vector is empty." << std::endl;
    }

    if (!Data_lengths.empty()) {
    max_data_length = *std::max_element(Data_lengths.begin(), Data_lengths.end());
    total_data_length = std::accumulate(Data_lengths.begin(), Data_lengths.end(), 0);
    } else {
        std::cout << "Data_lengths vector is empty." << std::endl;
    }
    reader.reset();

    std::cout << "Total Stripes: " << stripeCount << std::endl;
    std::cout << "Maximum number of rows in any stripe: " << max_stripe_rows << std::endl;
    std::cout << "Maximum data length: " << max_data_length << std::endl;
    std::cout << "Total data length: " << total_data_length << std::endl;

    uint8_t* data_in_HBM[BUFFERS_IN]; 
    uint8_t* data_out_HBM[BUFFERS_OUT];
    
    uint32_t max_input_size = max_data_length+PIPELINE_DEPTH+64;
    uint32_t max_output_size = max_stripe_rows; //MAX OUTPUT SIZE BYTES = (max_stripe_rows*4) , div 4 as data is divided in 4 ports 
    uint32_t max_track_size = max_stripe_rows*2; //max it can be 2x of the one data port size

    //Declare 256MB each
    //in ports
    for(int i = 0; i < BUFFERS_IN; i++)
    {
        data_in_HBM[i] = reinterpret_cast<uint8_t*>(aligned_alloc(ALIGNED_BYTES, max_input_size));
    }
    //out ports
    for (int i = 0; i < 8; ++i) {
        data_out_HBM[i] = reinterpret_cast<uint8_t*>(aligned_alloc(ALIGNED_BYTES, max_output_size));
    }
    //track ports
    for (int i = 8; i < BUFFERS_OUT; ++i) {
        data_out_HBM[i] = reinterpret_cast<uint8_t*>(aligned_alloc(ALIGNED_BYTES, max_track_size));
    }

   ///////Opening SSD////////
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
    ////////////////////////////

    if(FLAGS_bitstream != "") // FLAGS_bitstream != "" : Dont use for hw_emu/csim/sw_emu
    {
        /////////MY HOST///////////
            cl::Device device_;
            cl::Context context_;
            cl::CommandQueue cmd_;
            cl::Program program_;
            std::string My_device_name;
            std::map<int, cl::Kernel> kernels_;

            std::string target_device_name;
            std::vector<std::string> kernel_names;
            std::vector<int> kernel_arg_counts;
            int arg_count = 0;

            LOG(INFO) << "Loading Binaries From: " << FLAGS_bitstream << std::endl;
            cl::Program::Binaries binaries;
            {
                std::ifstream stream(FLAGS_bitstream, std::ios::binary);
                binaries = {{std::istreambuf_iterator<char>(stream),
                            std::istreambuf_iterator<char>()}};
            }
            const auto axlf_top = reinterpret_cast<const axlf*>(binaries.begin()->data());
            switch (axlf_top->m_header.m_mode) {
                case XCLBIN_FLAT:
                case XCLBIN_PR:
                case XCLBIN_TANDEM_STAGE2:
                case XCLBIN_TANDEM_STAGE2_WITH_PR:
                break;
                case XCLBIN_HW_EMU:
                setenv("XCL_EMULATION_MODE", "hw_emu", 0);
                break;
                case XCLBIN_SW_EMU:
                setenv("XCL_EMULATION_MODE", "sw_emu", 0);
                break;
                default:
                LOG(FATAL) << "Unknown xclbin mode";
            }
            target_device_name =
                reinterpret_cast<const char*>(axlf_top->m_header.m_platformVBNV);
            std::cout << "target_device_name: " << target_device_name << std::endl;
            if (auto metadata = xclbin::get_axlf_section(axlf_top, EMBEDDED_METADATA)) {
                TiXmlDocument doc;
                doc.Parse(
                    reinterpret_cast<const char*>(axlf_top) + metadata->m_sectionOffset,
                    nullptr, TIXML_ENCODING_UTF8);
                auto xml_core = doc.FirstChildElement("project")
                                    ->FirstChildElement("platform")
                                    ->FirstChildElement("device")
                                    ->FirstChildElement("core");
                std::string target_meta = xml_core->Attribute("target");
                for (auto xml_kernel = xml_core->FirstChildElement("kernel");
                    xml_kernel != nullptr;
                    xml_kernel = xml_kernel->NextSiblingElement("kernel")) 
                {
                    kernel_names.push_back(xml_kernel->Attribute("name"));
                    kernel_arg_counts.push_back(arg_count);
                    ++arg_count;
                    size_t i = kernel_names.size() - 1;
                    std::cout << "Kernel Name: " << kernel_names[i] << ", Argument Count: " << kernel_arg_counts[i] << std::endl;
                }
                if (target_meta == "hw_em") {
                setenv("XCL_EMULATION_MODE", "hw_emu", 0);
                } else if (target_meta == "csim") {
                setenv("XCL_EMULATION_MODE", "sw_emu", 0);
                }
            }
            else {
                LOG(FATAL) << "Cannot determine kernel name from binary";
            }
            if (const char* xcl_emulation_mode = getenv("XCL_EMULATION_MODE")) {
                LOG(FATAL) << "Cannot RUN EMU MODE"; 
            }
            else {
                LOG(INFO) << "Running on-board execution with Xilinx OpenCL";
            }

            std::vector<cl::Platform> platforms;
            CL_CHECK(cl::Platform::get(&platforms));
            cl_int err;
            for (const auto& platform : platforms) {
                std::string platformName = platform.getInfo<CL_PLATFORM_NAME>(&err);
                CL_CHECK(err);
                LOG(INFO) << "Found platform: " << platformName.c_str();
                if (platformName == "Xilinx") {
                    std::vector<cl::Device> devices;
                    CL_CHECK(platform.getDevices(CL_DEVICE_TYPE_ACCELERATOR, &devices));
                    for (const auto& device : devices) {
                        const std::string device_name = device.getInfo<CL_DEVICE_NAME>();
                        char bdf[32];
                        size_t bdf_size = 0;
                        CL_CHECK(clGetDeviceInfo(device.get(), CL_DEVICE_PCIE_BDF, sizeof(bdf), bdf,
                                                &bdf_size));
                        LOG(INFO) << "Found device: " << device_name;
                        if(device_name == target_device_name)
                        {
                            My_device_name = device_name;
                            device_ = device;
                            break;
                        }
                    }

                    LOG(INFO) << "Using " << My_device_name;
                    context_ = cl::Context(device_, nullptr, nullptr, nullptr, &err);
                    if (err == CL_DEVICE_NOT_AVAILABLE) {
                        LOG(WARNING) << "Device '" << My_device_name << "' not available";
                        continue;
                    }
                    CL_CHECK(err);
                    cmd_ = cl::CommandQueue(context_, device_, CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE | CL_QUEUE_PROFILING_ENABLE, &err);
                    CL_CHECK(err);
                    
                    std::vector<int> binary_status;
                    program_ =
                        cl::Program(context_, {device_}, binaries, &binary_status, &err);
                    for (auto status : binary_status) {
                        CL_CHECK(status);
                    }
                    CL_CHECK(err);
                    CL_CHECK(program_.build());
                    for (int i = 0; i < kernel_names.size(); ++i) {
                        // std::cout << "Kernels Count: " << kernel_arg_counts[i] << std::endl;
                        kernels_[kernel_arg_counts[i]] =
                            cl::Kernel(program_, kernel_names[i].c_str(), &err);
                        CL_CHECK(err);
                    }
                }
                else
                {
                    LOG(FATAL) << "Target platform 'Xilinx' not found";
                }
            }
            size_t map_size = kernels_.size();
            std::cout << "Kernels Size: " << map_size << std::endl;
            std::cout << "Kernel Programmed " << std::endl;
        ///////////////////////////

        ///////DECLARE BUFFERS FOR KERNEL////////
            cl::Buffer buffer_in_HBM[BUFFERS_IN];
            cl_mem_ext_ptr_t mIN_HBM[BUFFERS_IN];

            cl::Buffer buffer_out_HBM[BUFFERS_OUT];
            cl_mem_ext_ptr_t mOUT_HBM[BUFFERS_OUT];

            //HBM Bank location start from 16
            for(uint32_t i = 0; i < BUFFERS_IN; i ++)
            {
                mIN_HBM[i] = {XCL_MEM_TOPOLOGY | (unsigned int)(i+16), data_in_HBM[i], 0};
                buffer_in_HBM[i] = cl::Buffer(context_, CL_MEM_EXT_PTR_XILINX | CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                                (size_t)(max_input_size), &mIN_HBM[i], &err);     // CL_MEM_WRITE_ONLY, CL_MEM_READ_ONLY, CL_MEM_READ_WRITE
                CL_CHECK(err);
            }

            for(int i = 0; i < 8; i++)
            {
                // (XCL_MEM_TOPOLOGY | memory bank)
                mOUT_HBM[i] = {XCL_MEM_TOPOLOGY | (unsigned int)(i+2+16), data_out_HBM[i], 0};
                buffer_out_HBM[i] = cl::Buffer(context_, CL_MEM_EXT_PTR_XILINX | CL_MEM_USE_HOST_PTR | CL_MEM_WRITE_ONLY,
                                (size_t)(max_output_size), &mOUT_HBM[i], &err);     // CL_MEM_WRITE_ONLY, CL_MEM_READ_ONLY, CL_MEM_READ_WRITE
                CL_CHECK(err);
            }

            for(int i = 8; i < BUFFERS_OUT; i++)
            {
                // (XCL_MEM_TOPOLOGY | memory bank)
                mOUT_HBM[i] = {XCL_MEM_TOPOLOGY | (unsigned int)(i+2+16), data_out_HBM[i], 0};
                buffer_out_HBM[i] = cl::Buffer(context_, CL_MEM_EXT_PTR_XILINX | CL_MEM_USE_HOST_PTR | CL_MEM_WRITE_ONLY,
                                (size_t)(max_track_size), &mOUT_HBM[i], &err);     // CL_MEM_WRITE_ONLY, CL_MEM_READ_ONLY, CL_MEM_READ_WRITE
                CL_CHECK(err);
            }

            std::cout << "Data in buffer size(MB): " << (max_data_length / (1024.0 * 1024.0)) << std::endl;
            std::cout << "Data out buffer size(MB): " << (max_output_size / (1024.0 * 1024.0)) << std::endl;

            cl::Kernel kernelDD;
            KRNL_file_size_bytes = Data_lengths[0];
            file_size_rem = KRNL_file_size_bytes%64;
            if(file_size_rem!=0)
            {
                KRNL_file_size_bytes = KRNL_file_size_bytes + (64 - file_size_rem);
            }

            KRNL_file_size_bytes = (KRNL_file_size_bytes + PIPELINE_DEPTH);  //the pipeline depth of FPGA 64*8=512 + 64 = 576
            KRNL_file_size_bytes = KRNL_file_size_bytes/64;

            for (const auto& kvp : kernels_) {
                int index = kvp.first; // Get the index (key) of the kernel
                std::cout << "Setting Kernel["<<index<<"] Arg" << std::endl;
                kernelDD = kvp.second; // Get the kernel associated with the index (key)
                kernelDD.setArg(0, buffer_in_HBM[0]);
                kernelDD.setArg(1, buffer_out_HBM[0]);
                kernelDD.setArg(2, buffer_out_HBM[2]);
                kernelDD.setArg(3, buffer_out_HBM[4]);
                kernelDD.setArg(4, buffer_out_HBM[6]);
                kernelDD.setArg(5, buffer_out_HBM[8]);
                kernelDD.setArg(6, sizeof(wait_count), &wait_count);
                kernelDD.setArg(7, sizeof(KRNL_file_size_bytes), &KRNL_file_size_bytes);
            }
            std::cout << "Kernels Argument Set." << std::endl;
            
        int ret_aio = 0;
        struct aiocb aio_rf;
        struct aiocb aio_rf1;
        ///////Launching KERNEL SINGLE SHOT////////
            std::vector<cl::Event> kernel_events(3);
            std::vector<cl::Event> kernel_wait_events;

            memset(data_in_HBM[0], 0, max_input_size);
            memset(data_in_HBM[1], 0, max_input_size);

            async_readnorm(&aio_rf, (void *)(data_in_HBM[0]), nvmeFd, Data_lengths[0], Data_offsets[0]); 
            while( aio_error(&aio_rf) == EINPROGRESS ) {;}
            ret_aio = aio_return (&aio_rf);
            printf("Bytes Read. %d \n", ret_aio);

            CL_CHECK(cmd_.enqueueWriteBuffer(buffer_in_HBM[0], CL_FALSE, 0, max_input_size, data_in_HBM[0], nullptr, &kernel_events[0]));
            // CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_in_HBM[0])} , 0 , nullptr, &kernel_events[0]));  //DRAM FPGA
            kernel_wait_events.resize(0);
            CL_CHECK(cmd_.flush());
            CL_CHECK(cmd_.finish());
            std::cout << "C2F Done" << std::endl;

            kernelDD.setArg(0, buffer_in_HBM[0]);
            kernelDD.setArg(1, buffer_out_HBM[0]);
            kernelDD.setArg(2, buffer_out_HBM[2]);
            kernelDD.setArg(3, buffer_out_HBM[4]);
            kernelDD.setArg(4, buffer_out_HBM[6]);
            kernelDD.setArg(5, buffer_out_HBM[8]);  ///6,7 already set use old
            kernelDD.setArg(6, sizeof(wait_count), &wait_count);
            kernelDD.setArg(7, sizeof(KRNL_file_size_bytes), &KRNL_file_size_bytes);

            CL_CHECK(cmd_.flush());
            CL_CHECK(cmd_.finish());
            std::cout << "Kernel Arg Set" << std::endl;

            kernel_wait_events.push_back(kernel_events[0]);
            CL_CHECK(cmd_.enqueueTask(kernelDD, &kernel_wait_events, &kernel_events[1]));
            kernel_wait_events.resize(0);
            kernel_wait_events.push_back(kernel_events[1]);
            CL_CHECK(cmd_.flush());
            CL_CHECK(cmd_.finish());
            std::cout << "Kernel Done" << std::endl;

            //CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED,  CL_MIGRATE_MEM_OBJECT_HOST     
            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[0])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                        &kernel_wait_events, &kernel_events[2]));
            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[2])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                        &kernel_wait_events, &kernel_events[2]));
            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[4])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                        &kernel_wait_events, &kernel_events[2]));
            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[6])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                        &kernel_wait_events, &kernel_events[2]));
            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[8])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                        &kernel_wait_events, &kernel_events[2]));

            CL_CHECK(cmd_.flush());
            CL_CHECK(cmd_.finish());
            std::cout << "F2C Done" << std::endl;
            std::cout << "Initial Kernels Finished" << std::endl;

            int64_t load_time_ns = 0;
            int64_t compute_time_ns = 0; 
            int64_t store_time_ns = 0;
            double load_gbps = 0;
            double store_gbps = 0;

            ///////KERNEL Profiling////////
                cl_ulong start, end;
                kernel_events[0].getProfilingInfo(CL_PROFILING_COMMAND_START, &start);
                kernel_events[0].getProfilingInfo(CL_PROFILING_COMMAND_END, &end);
                load_time_ns = (end - start); //-- actual time is reported in nanoseconds

                kernel_events[1].getProfilingInfo(CL_PROFILING_COMMAND_START, &start);
                kernel_events[1].getProfilingInfo(CL_PROFILING_COMMAND_END, &end);
                compute_time_ns = (end - start); //-- actual time is reported in nanoseconds

                kernel_events[2].getProfilingInfo(CL_PROFILING_COMMAND_START, &start);
                kernel_events[2].getProfilingInfo(CL_PROFILING_COMMAND_END, &end);
                store_time_ns = (end - start); //-- actual time is reported in nanoseconds
            ///////////////////////////
        #ifdef PRINT_DEBUG
            std::cout << "Kernel Exec time(ms): " << (compute_time_ns) * 1e-6 << std::endl;
            std::cout << "Data Input Size (MB): " << (float)(Data_lengths[0]/(1024.0*1024.0)) << std::endl;
            std::cout << "Data Output Size (MB): " << (float)((stripe_rows[0]*4)/(1024.0*1024.0)) << std::endl;
            std::cout << "Kernel throughput(GB/s): " << (float)(Data_lengths[0])/(float)(compute_time_ns) << std::endl;

            std::cout << "CPU-2-FPGA Transfer time(ms): " << (load_time_ns) * 1e-6 << std::endl;
            std::cout << "FPGA-2-CPU Transfer time(ms): " << (store_time_ns) * 1e-6 << std::endl;
        #endif
        ///////Launching KERNEL DATAFLOW////////
            //non multiple RL adjustment
            // stripeCount -= 1;    //remove last stripe

            uint32_t NITERS = stripeCount + 3;  //IO, C2F, FCOMP, F2C

            cl_uint one = 1;
            std::vector<cl::Event> C2F_events(NITERS);
            std::vector<cl::Event> Comp_events(NITERS);
            std::vector<cl::Event> F2C_events(NITERS*10);
            std::vector<cl::Event> kernel_wait_events0;
            std::vector<cl::Event> kernel_wait_events1;

            auto asyncTimeS = std::chrono::steady_clock::now();
            auto asyncTimeE = std::chrono::steady_clock::now();
            auto asyncTime = std::chrono::duration_cast<std::chrono::microseconds>(asyncTimeE - asyncTimeS);

            auto readTimeS = std::chrono::steady_clock::now();
            auto readTimeE = std::chrono::steady_clock::now();
            auto readTime = std::chrono::duration_cast<std::chrono::microseconds>(asyncTimeE - asyncTimeS);

            auto FPGATimeS = std::chrono::steady_clock::now();
            auto FPGATimeE = std::chrono::steady_clock::now();
            auto FPGATime = std::chrono::duration_cast<std::chrono::microseconds>(asyncTimeE - asyncTimeS);

            auto wrTimeS = std::chrono::steady_clock::now();
            auto wrTimeE = std::chrono::steady_clock::now();
            auto wrTime = std::chrono::duration_cast<std::chrono::microseconds>(asyncTimeE - asyncTimeS);

            auto tempTimeA = std::chrono::steady_clock::now();
            auto tempTimeB = std::chrono::steady_clock::now();
            auto tempTime = std::chrono::duration_cast<std::chrono::microseconds>(tempTimeA - tempTimeB);

            double time_fpga[NITERS] = {0.0F};
            double time_read[NITERS] = {0.0F};
            double time_wr[NITERS] = {0.0F};
            double time_async[NITERS] = {0.0F};
            // async_readnorm(void* data_in, int nvmeFd, int vector_size_bytes, int offset)
            // std::cout << "Starting DF, Total Iters: " << NITERS << std::endl;
            // std::cout << "stripeCount: " << stripeCount << std::endl;

            auto dfstart = std::chrono::steady_clock::now();
            if(dataflow)
            {
                std::cout << "***Dataflow Implementation***" << std::endl;
                dfstart = std::chrono::steady_clock::now();
                for (int i = 0; i < NITERS; i++)
                {
                    // std::cout << "ITER COUNT: " << i << std::endl;
                    asyncTimeS = std::chrono::steady_clock::now();
                    //IO READ
                    if(i < stripeCount)
                    {
                        if((i%2) == 0)
                        {
                            async_readnorm(&aio_rf, (void *)(data_in_HBM[0]), nvmeFd, Data_lengths[i], Data_offsets[i]); 
                            // std::cout << "IO_E" << std::endl;
                        }
                        else
                        {
                            async_readnorm(&aio_rf1, (void *)(data_in_HBM[1]), nvmeFd, Data_lengths[i], Data_offsets[i]); 
                            // std::cout << "IO_O" << std::endl;
                        }
                    }   

                    // tempTimeA = std::chrono::steady_clock::now();
                    //CPU_2_FPGA
                    if((i >= 1) && (i < (stripeCount+1)))
                    {
                        int Ssize = Data_lengths[i-1]+PIPELINE_DEPTH+64;
                        if(((i-1)%2) == 0)
                        {
                            CL_CHECK(cmd_.enqueueWriteBuffer(buffer_in_HBM[0], CL_FALSE, 0, Ssize, data_in_HBM[0], nullptr, &C2F_events[i-1]));
                            // cmd_.enqueueMigrateMemObjects({(buffer_in_HBM[0])} , 0 , nullptr, &C2F_events[i-1]);
                            // std::cout << "C2F_E" << std::endl;
                        }
                        else
                        {
                            CL_CHECK(cmd_.enqueueWriteBuffer(buffer_in_HBM[1], CL_FALSE, 0, Ssize, data_in_HBM[1], nullptr, &C2F_events[i-1]));
                            // cmd_.enqueueMigrateMemObjects({(buffer_in_HBM[1])} , 0 , nullptr, &C2F_events[i-1]);
                            // std::cout << "C2F_O" << std::endl;
                        }
                        // std::cout << "C2F" << ":" << i-1 << std::endl;
                    }
                    // tempTimeB = std::chrono::steady_clock::now();
                    // tempTime = std::chrono::duration_cast<std::chrono::microseconds>(tempTimeB - tempTimeA);
                    // std::cout << "time_async C2F: " << static_cast<double>(tempTime.count()) <<std::endl;

                    //KERNEL CALL
                    if((i >= 2) && (i < (stripeCount+2)))
                    {
                        KRNL_file_size_bytes = Data_lengths[i-2];
                        file_size_rem = KRNL_file_size_bytes%64;
                        if(file_size_rem!=0)
                        {
                            KRNL_file_size_bytes = KRNL_file_size_bytes + (64 - file_size_rem);
                        }
                        KRNL_file_size_bytes = (KRNL_file_size_bytes + PIPELINE_DEPTH);  //the pipeline depth of FPGA 64*8=512 + 64 = 576
                        KRNL_file_size_bytes = KRNL_file_size_bytes/64;

                        if(((i-2)%2) == 0)
                        {
                            //Set Arg
                                kernelDD.setArg(0, buffer_in_HBM[0]);
                                kernelDD.setArg(1, buffer_out_HBM[0]);
                                kernelDD.setArg(2, buffer_out_HBM[2]);
                                kernelDD.setArg(3, buffer_out_HBM[4]);
                                kernelDD.setArg(4, buffer_out_HBM[6]);
                                kernelDD.setArg(5, buffer_out_HBM[8]);
                                kernelDD.setArg(6, sizeof(wait_count), &wait_count);
                                kernelDD.setArg(7, sizeof(KRNL_file_size_bytes), &KRNL_file_size_bytes);
                            //Kernel Call
                                // std::cout << "COMP_E" << std::endl;
                        }
                        else
                        {
                            //Set Arg
                                kernelDD.setArg(0, buffer_in_HBM[1]);
                                kernelDD.setArg(1, buffer_out_HBM[1]);
                                kernelDD.setArg(2, buffer_out_HBM[3]);
                                kernelDD.setArg(3, buffer_out_HBM[5]);
                                kernelDD.setArg(4, buffer_out_HBM[7]);
                                kernelDD.setArg(5, buffer_out_HBM[9]);
                                kernelDD.setArg(6, sizeof(wait_count), &wait_count);
                                kernelDD.setArg(7, sizeof(KRNL_file_size_bytes), &KRNL_file_size_bytes);
                            //Kernel Call
                                // std::cout << "COMP_O" << std::endl;
                        }
                        CL_CHECK(cmd_.enqueueTask(kernelDD, nullptr, &Comp_events[i-2]));
                        // std::cout << "COMP" << ":" << i-2 << std::endl;
                    }

                    //FPGA_2_CPU
                    if((i >= 3) && (i < (stripeCount+3)))
                    {
                        if(((i-3)%2) == 0)
                        {
                            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[0])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                            nullptr, &F2C_events[((i-3)*10)+0]));
                            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[2])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                            nullptr, &F2C_events[((i-3)*10)+1]));
                            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[4])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                            nullptr, &F2C_events[((i-3)*10)+2]));
                            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[6])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                            nullptr, &F2C_events[((i-3)*10)+3]));
                            // cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[8])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                            //                                 nullptr, &F2C_events[((i-3)*10)+4]);
                            // std::cout << "F2C_E" << std::endl;
                        }
                        else
                        {
                            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[1])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                            nullptr, &F2C_events[((i-3)*10)+5]));
                            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[3])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                            nullptr, &F2C_events[((i-3)*10)+6]));
                            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[5])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                            nullptr, &F2C_events[((i-3)*10)+7]));
                            CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[7])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                            nullptr, &F2C_events[((i-3)*10)+8]));
                            // cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[9])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                            //                                 nullptr, &F2C_events[((i-3)*10)+9]);
                            // std::cout << "F2C_O" << std::endl;
                        }

                        // std::cout << "F2C" << ":" << i-3 << std::endl;
                    }
                    asyncTimeE = std::chrono::steady_clock::now();
                    asyncTime = std::chrono::duration_cast<std::chrono::microseconds>(asyncTimeE - asyncTimeS);
                    time_async[i] = static_cast<double>(asyncTime.count());

                    ///WAITS///
                    
                    //IO READ
                    readTimeS = std::chrono::steady_clock::now();
                    if(i < stripeCount)
                    {
                        int ret = 0;
                        if(i%2 == 0)
                        {
                            while( aio_error(&aio_rf) == EINPROGRESS ) {;}
                            ret = aio_return (&aio_rf);
                            if(ret <= 0)
                            {
                                std::cerr << "Read Error. Bytes Read: " << ret << std::endl;
                            }
                            readTimeE = std::chrono::steady_clock::now();
                            wrTimeS = std::chrono::steady_clock::now();
                            writeZeros(data_in_HBM[0], Data_lengths[i], PIPELINE_DEPTH);
                        }
                        else
                        {
                            while( aio_error(&aio_rf1) == EINPROGRESS ) {;}
                            ret = aio_return (&aio_rf1);
                            if(ret <= 0)
                            {
                                std::cerr << "Read Error. Bytes Read: " << ret << std::endl;
                            }
                            readTimeE = std::chrono::steady_clock::now();
                            wrTimeS = std::chrono::steady_clock::now();
                            writeZeros(data_in_HBM[1], Data_lengths[i], PIPELINE_DEPTH);
                        }
                        // std::cout << "Read Bytes: " << ret << std::endl;
                    }   
                    // readTimeE = std::chrono::steady_clock::now();
                    wrTimeE = std::chrono::steady_clock::now();
                    wrTime = std::chrono::duration_cast<std::chrono::microseconds>(wrTimeE - wrTimeS);
                    readTime = std::chrono::duration_cast<std::chrono::microseconds>(readTimeE - readTimeS);
                    time_read[i] = static_cast<double>(readTime.count());
                    time_wr[i] = static_cast<double>(wrTime.count());

                    FPGATimeS = std::chrono::steady_clock::now();
                    //CPU_2_FPGA
                    if((i >= 1) && (i < (stripeCount+1)))
                    {
                        CL_CHECK(clWaitForEvents(one,(cl_event*)(&C2F_events[i-1])));
                        // std::cout << "C2F Wait" << ":" << i-1 << std::endl;
                    }                    
                    
                    //KERNEL CALL
                    if((i >= 2) && (i < (stripeCount+2)))
                    {
                        CL_CHECK(clWaitForEvents(one,(cl_event*)(&Comp_events[i-2])));
                        // std::cout << "COMP Wait" <<":" << i-2 << std::endl;
                    }

                    //FPGA_2_CPU
                    if((i >= 3) && (i < (stripeCount+3)))
                    {
                        if(((i-3)%2) == 0)
                        {
                            CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+0])));
                            CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+1])));
                            CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+2])));
                            CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+3])));
                            // clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+4]));
                        }
                        else
                        {
                            CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+5])));
                            CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+6])));
                            CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+7])));
                            CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+8])));
                            // clWaitForEvents(one,(cl_event*)(&F2C_events[((i-3)*10)+9]));
                        }
                        // std::cout << "F2C Wait" << ":" << i-3 << std::endl;
                    }
                    FPGATimeE = std::chrono::steady_clock::now();
                    FPGATime = std::chrono::duration_cast<std::chrono::microseconds>(FPGATimeE - FPGATimeS);
                    time_fpga[i] = static_cast<double>(FPGATime.count());

                    //Data Verification
                    // if((i >= 3) && (i < (stripeCount+3)))
                    // {
                    //     if(((i-3)%2) == 0)
                    //     {
                    //         verif_all(reinterpret_cast<uint8_t*>(data_out_HBM[0]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[2]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[4]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[6]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[8]));
                    //     }
                    //     else
                    //     {
                    //         verif_all(reinterpret_cast<uint8_t*>(data_out_HBM[1]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[3]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[5]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[7]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[9]));
                    //     }
                    // }
                }
            }
            else
            {
                std::cout << "***Sequential Implementation***" << std::endl;
                dfstart = std::chrono::steady_clock::now();
                for (int i = 0; i < stripeCount; i++)
                {
                    std::cout << "ITER COUNT: " << i << std::endl;
                    //IO READ
                    memset(data_in_HBM[0], 0, max_input_size);
                    async_readnorm(&aio_rf, (void *)(data_in_HBM[0]), nvmeFd, Data_lengths[i], Data_offsets[i]);  
                    while( aio_error(&aio_rf) == EINPROGRESS ) {;}
                    int ret = aio_return (&aio_rf);
                    if(ret <= 0)
                    {
                        std::cerr << "Read Error. Bytes Read: " << ret << std::endl;
                    }

                    //CPU_2_FPGA
                    int Ssize = Data_lengths[i]+PIPELINE_DEPTH+64;
                    CL_CHECK(cmd_.enqueueWriteBuffer(buffer_in_HBM[0], CL_FALSE, 0, Ssize, data_in_HBM[0], nullptr, &C2F_events[i]));
                    CL_CHECK(clWaitForEvents(one,(cl_event*)(&C2F_events[i])));

                    //KERNEL CALL
                    KRNL_file_size_bytes = Data_lengths[i];
                    file_size_rem = KRNL_file_size_bytes%64;
                    if(file_size_rem!=0)
                    {
                        KRNL_file_size_bytes = KRNL_file_size_bytes + (64 - file_size_rem);
                    }
                    KRNL_file_size_bytes = (KRNL_file_size_bytes + PIPELINE_DEPTH);  //the pipeline depth of FPGA 64*8=512 + 64 = 576
                    KRNL_file_size_bytes = KRNL_file_size_bytes/64;
                    kernelDD.setArg(0, buffer_in_HBM[0]);
                    kernelDD.setArg(1, buffer_out_HBM[0]);
                    kernelDD.setArg(2, buffer_out_HBM[2]);
                    kernelDD.setArg(3, buffer_out_HBM[4]);
                    kernelDD.setArg(4, buffer_out_HBM[6]);
                    kernelDD.setArg(5, buffer_out_HBM[8]);
                    kernelDD.setArg(6, sizeof(wait_count), &wait_count);
                    kernelDD.setArg(7, sizeof(KRNL_file_size_bytes), &KRNL_file_size_bytes);
                    CL_CHECK(cmd_.enqueueTask(kernelDD, nullptr, &Comp_events[i]));
                    CL_CHECK(clWaitForEvents(one,(cl_event*)(&Comp_events[i])));


                    //FPGA_2_CPU
                    CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[0])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                    nullptr, &F2C_events[((i)*10)+0]));
                    CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[2])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                    nullptr, &F2C_events[((i)*10)+1]));
                    CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[4])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                    nullptr, &F2C_events[((i)*10)+2]));
                    CL_CHECK(cmd_.enqueueMigrateMemObjects({(buffer_out_HBM[6])}, CL_MIGRATE_MEM_OBJECT_HOST , 
                                                    nullptr, &F2C_events[((i)*10)+3]));

                    CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i)*10)+0])));
                    CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i)*10)+1])));
                    CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i)*10)+2])));
                    CL_CHECK(clWaitForEvents(one,(cl_event*)(&F2C_events[((i)*10)+3])));

                    //Data Verification
                    // if((i >= 3) && (i < (stripeCount+3)))
                    // {
                    //     if(((i-3)%2) == 0)
                    //     {
                    //         verif_all(reinterpret_cast<uint8_t*>(data_out_HBM[0]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[2]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[4]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[6]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[8]));
                    //     }
                    //     else
                    //     {
                    //         verif_all(reinterpret_cast<uint8_t*>(data_out_HBM[1]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[3]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[5]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[7]), 
                    //                 reinterpret_cast<uint8_t*>(data_out_HBM[9]));
                    //     }
                    // }
                }
            }
            auto dfend = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(dfend - dfstart);
            double total_time = static_cast<double>(duration.count());
            std::cout << "Schedule Completed." << std::endl;

            (void)close(nvmeFd);


            ////PROFILING RESULTS////
            std::cout << std::dec << "----CPU TIMER BASED CALCULATIONS----" << std::endl;
            std::cout << "TOTAL ITERATION COUNT: " << stripeCount << std::endl;
            std::cout << "END 2 END exec Total Time (ns): " << total_time << std::endl;
            float total_read = 0.0;
            
            #ifdef PRINT_DEBUG
                if(dataflow && (NITERS < 20))
                {
                    for (int s = 0; s < NITERS; s++)
                    {
                        total_read += time_async[s];
                        total_read += time_read[s];
                        total_read += time_fpga[s];
                        
                        std::cout << std::dec << "Total Async Call[" <<s<< "] Time (us): " << time_async[s] << std::endl;
                        if(s < stripeCount)
                        {
                            std::cout << std::dec << "Total Read[" <<s<< "] Time (us): " << time_read[s] << std::endl;
                            std::cout << std::dec << "Total Write0s[" <<s<< "] Time (us): " << time_wr[s] << std::endl;
                        }
                        std::cout << std::dec << "Total FPGA[" <<s<< "] Time (us): " << time_fpga[s] << std::endl;
                    }
                    std::cout << std::dec << "Total Read Sum(us): " << total_read << std::endl;
                }
            #endif

            std::cout << std::dec << "Encoded Data File Size(MB): " << ((float)(total_data_length)/(float)(1024.0*1024.0)) << std::endl;
            std::cout << std::dec << "Total Output Size(MB): " << ((float)((nrows)*4)/(float)(1024.0*1024.0)) << std::endl;
            std::cout << std::dec << "BANDWIDTH INPUT (GB/s): " << ((double)(total_data_length))/(double)(total_time) << std::endl;
            std::cout << std::dec << "BANDWIDTH OUTPUT (GB/s): " << ((double)((nrows)*4))/(double)(total_time) << std::endl;
        
        
        // After using the buffers, free the allocated memory
        for(int i = 0; i < BUFFERS_IN; i++) {
            free(data_in_HBM[i]);
        }

        for (int i = 0; i < BUFFERS_OUT; ++i) {
            free(data_out_HBM[i]);
        }

        //////////////////////////
    }
    else
    {
        //run csim, sw_emu
        std::cout << "***CSIM/SW_EMU MODE RUN Single Stripe Single Column for Testing***" << std::endl;
        // wait_count = WAIT_MAX;
    }  
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