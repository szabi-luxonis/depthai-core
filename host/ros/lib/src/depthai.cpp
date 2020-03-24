#include <exception>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include <boost/algorithm/string/replace.hpp>

#include "device_support_listener.hpp"
#include "host_data_packet.hpp"
#include "host_data_reader.hpp"
#include "nnet/tensor_info.hpp"
#include "nnet/tensor_info_helper.hpp"
#include "nnet/tensor_entry.hpp"
#include "nnet/nnet_packet.hpp"
#include "nnet/tensor_entry_container.hpp"
#include "pipeline/host_pipeline.hpp"
#include "pipeline/host_pipeline_config.hpp"
#include "pipeline/cnn_host_pipeline.hpp"
#include "disparity_stream_post_processor.hpp"
#include "cnn_info.hpp"
#include "depthai_constants.hpp"
#include "json_helper.hpp"
#include "version.hpp"
#include "xlink/xlink_wrapper.hpp"
#include "test_data_subject.hpp"
#include "host_json_helper.hpp"

#include "depthai.hpp"

struct DepthAI::DepthAI_Impl
{
    XLinkGlobalHandler_t xlinkGlobalHandler =
    {
        .profEnable = 0,  
        .profilingData = {
            .totalReadTime   = 0.f,
            .totalWriteTime  = 0.f,
            .totalReadBytes  = 0,
            .totalWriteBytes = 0,
            .totalBootCount  = 0,
            .totalBootTime   = 0.f
        },
        .loglevel   = 0,
        .protocol   = USB_VSC
    };

    XLinkHandler_t xlinkDeviceHandler =
    {
        .devicePath  = NULL,
        .devicePath2 = NULL,
        .linkId      = 0
    };

    boost::scoped_ptr<XLinkWrapper> xlink;
    json config_d2h;
    boost::scoped_ptr<CNNHostPipeline> pipeline;
    boost::scoped_ptr<DisparityStreamPostProcessor> disparityPostProc;
    boost::scoped_ptr<DeviceSupportListener>        deviceSupportListener;

    bool initDevice(
        const std::string &device_cmd_file
    )
    {
        bool result = false;

        do
        {
            // xlink
            if (nullptr != xlink)
            {
                std::cout << "Device is already initialized" << "\n";
                break;
            }

            xlink.reset(new XLinkWrapper(true));

            if (!xlink->initFromHostSide(
                    &xlinkGlobalHandler,
                    &xlinkDeviceHandler,
                    device_cmd_file,
                    true)
                )
            {
                std::cout << "Error initializing xlink\n";
                break;
            }


            // config_d2h
            {
                printf("Loading config file\n");

                std::string config_d2h_str;
                StreamInfo si("config_d2h", 102400);

                int config_file_length = xlink->openReadAndCloseStream(
                        si,
                        config_d2h_str
                        );

                try
                {
                    getJSONFromString(config_d2h_str, config_d2h);
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Error: Cant parse config_d2h:" << e.what() << '\n'
                        << "config_d2h :" << config_d2h_str << "\n";
                    break;
                }
            }


            // check version
            {
                std::string device_version = config_d2h.at("_version").get<std::string>();
                if (device_version != c_depthai_version)
                {
                    printf("Version does not match (%s & %s)\n",
                        device_version.c_str(), c_depthai_version);
                    break;
                }

                std::string device_dev_version = config_d2h.at("_dev_version").get<std::string>();
                if (device_dev_version != c_depthai_dev_version)
                {
                    printf("WARNING: Version (dev) does not match (%s & %s)\n",
                        device_dev_version.c_str(), c_depthai_dev_version);
                }
            }

            // device support listener                
            deviceSupportListener.reset(new DeviceSupportListener);
           
            deviceSupportListener->observe(
                    *xlink.get(),
                    c_streams_myriad_to_pc.at("meta_d2h")
                    );

            result = true;
        } while (false);

        if (!result)
        {
            xlink.reset();
        }

        return result;
    }


    std::vector<std::string> getAvailableSteams()
    {
        std::vector<std::string> result;

        if (config_d2h.is_object() &&
            config_d2h.contains("_available_streams") &&
            config_d2h.at("_available_streams").is_array()
            )
        {
            for (const auto &obj : config_d2h.at("_available_streams"))
            {
                result.push_back(obj.get<std::string>());
            }
        }

        return result;
    }

    bool createPipeline(
        const std::string &config_json_str
    )
    {
        bool result = false;

        do
        {
            // check xlink
            if (nullptr == xlink)
            {
                std::cout << "Device is not initialized\n";
                break;
            }

            // str -> json
            json config_json;
            try
            {
                getJSONFromString(config_json_str, config_json);
            
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error: Cant parse config_json:" << e.what() << "\n"
                    << "config_json :" << config_json_str << "\n";
                break;
            }

            HostPipelineConfig config;

            try
            {
                // json -> configurations 
                if (!config.initWithJSON(config_json))
                {
                    std::cout << "Error: Cant init configs with json: " << config_json.dump() << "\n";
                    break;
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                break;
            }

            std::vector<TensorInfo>       tensors_info;
            
            try
            {
                if (parseTensorInfosFromJsonFile(config.ai.blob_file_config, tensors_info))
                {
                    std::cout << "CNN configurations read: " << config.ai.blob_file_config.c_str() << "\n";
                }
                else
                {
                    std::cout << "There is no cnn configuration file or error in it\'s parsing: " << config.ai.blob_file_config.c_str() << "\n";
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
            // pipeline configurations json
            // homography
            std::vector<float> homography_buff = {
                // default for BW0250TG:
                9.8806816e-01,  2.9474013e-03,  5.0676174e+00,
                -8.7650679e-03,  9.9214733e-01, -8.7952757e+00,
                -8.4495878e-06, -3.6034894e-06,  1.0000000e+00
            };

            if (config.depth.calibration_file.empty())
            {
                std::cout << "depthai: Calibration file is not specified, will use default setting;\n";
            }
            else
            {
                HostDataReader calibration_reader;
                if (!calibration_reader.init(config.depth.calibration_file))
                {
                    std::cout << "depthai: Error opening calibration file: " << config.depth.calibration_file << "\n";
                    break;
                }

                int sz = calibration_reader.getSize();
                assert(sz == sizeof(float) * 9);
                std::cout << "Read: " << calibration_reader.readData(reinterpret_cast<unsigned char*>(homography_buff.data()), sz) << std::endl;
            }

            json json_config_obj;
            json_config_obj["board"]["swap-left-and-right-cameras"] = config.board_config.swap_left_and_right_cameras;
            json_config_obj["board"]["left_fov_deg"] = config.board_config.left_fov_deg;
            json_config_obj["board"]["left_to_right_distance_m"] = config.board_config.left_to_right_distance_m;
            json_config_obj["board"]["left_to_rgb_distance_m"] = config.board_config.left_to_rgb_distance_m;
            json_config_obj["_board"] =
            {
                {"_homography_right_to_left", homography_buff}
            };
            json_config_obj["depth"]["padding_factor"] = config.depth.padding_factor;

            json_config_obj["_load_inBlob"] = true;
            json_config_obj["_pipeline"] =
            {
                {"_streams", json::array()}
            };

            json_config_obj["ai"]["calc_dist_to_bb"] = config.ai.calc_dist_to_bb;

            bool add_disparity_post_processing_color = false;
            bool add_disparity_post_processing_mm = false;
            std::vector<std::string> pipeline_device_streams;

            for (const auto &stream : config.streams)
            {
                if (stream.name == "depth_color_h")
                {
                    add_disparity_post_processing_color = true;
                    json obj = { {"name", "disparity"} };
                    json_config_obj["_pipeline"]["_streams"].push_back(obj);
                }
                else if (stream.name == "depth_mm_h")
                {
                    add_disparity_post_processing_mm = true;
                    json obj = { {"name", "disparity"} };
                    json_config_obj["_pipeline"]["_streams"].push_back(obj);
                }
                else
                {
                    json obj = { {"name" ,stream.name} };

                    if (!stream.data_type.empty()) { obj["data_type"] = stream.data_type; };
                    if (0.f != stream.max_fps)     { obj["max_fps"]   = stream.max_fps;   };

                    // TODO: temporary solution
                    if (stream.name == "depth_sipp")
                            // {
                            //     obj["data_type"] = "uint8";
                            //     c_streams_myriad_to_pc["depth_sipp"] = StreamInfo("depth_sipp",     0, { 720, 1280}  );
                            // }
                            {
                                obj["data_type"] = "uint16";
                                c_streams_myriad_to_pc["depth_sipp"] = StreamInfo("depth_sipp",     0, { 720, 1280}, 2  );
                            }
                            // {
                            //     obj["data_type"] = "rgb";
                            //     c_streams_myriad_to_pc["depth_sipp"] = StreamInfo("depth_sipp",     2764800, { 720, 1280, 3} );
                            // }

                    json_config_obj["_pipeline"]["_streams"].push_back(obj);
                    pipeline_device_streams.push_back(stream.name);
                }
            }

            // host -> "config_h2d" -> device
            std::string pipeline_config_str_packed = json_config_obj.dump();
            std::cout << "config_h2d json:\n" << pipeline_config_str_packed << "\n";
            // resize, as xlink expects exact;y the same size for input:
            assert(pipeline_config_str_packed.size() < g_streams_pc_to_myriad.at("config_h2d").size);
            pipeline_config_str_packed.resize(g_streams_pc_to_myriad.at("config_h2d").size, 0);

            if (!xlink->openWriteAndCloseStream(
                    g_streams_pc_to_myriad.at("config_h2d"),
                    pipeline_config_str_packed.data())
                )
            {
                std::cout << "depthai: pipelineConfig write error;\n";
                break;
            }

            // read & pass blob file
            if (config.ai.blob_file.empty())
            {
                std::cout << "depthai: Blob file is not specified, will use default setting;\n";
            }
            else
            {
                HostDataReader _blob_reader;
                if (!_blob_reader.init(config.ai.blob_file))
                {
                    std::cout << "depthai: Error opening blob file: " << config.ai.blob_file << "\n";
                    break;
                }
                int size_blob = _blob_reader.getSize();

                std::vector<uint8_t> buff_blob(size_blob);

                std::cout << "Read: " << _blob_reader.readData(buff_blob.data(), size_blob) << std::endl;

                // inBlob
                StreamInfo blobInfo;
                blobInfo.name = "inBlob";
                blobInfo.size = size_blob;

                if (!xlink->openWriteAndCloseStream(blobInfo, buff_blob.data()))
                {
                    std::cout << "depthai: pipelineConfig write error;\n";
                    break;
                }
                printf("depthai: done sending Blob file %s\n", config.ai.blob_file.c_str());

                // outBlob
                StreamInfo outBlob;
                outBlob.name = "outBlob";
                //TODO: remove asserts considering StreamInfo size
                outBlob.size = 1;

                cnn_info cnn_input_info;

                static char cnn_info_arr[sizeof(cnn_info)];
                xlink->openReadAndCloseStream(
                    outBlob,
                    (void*)cnn_info_arr,
                    sizeof(cnn_info)
                    );

                memcpy(&cnn_input_info, &cnn_info_arr, sizeof(cnn_input_info));

                printf("CNN input width: %d\n", cnn_input_info.cnn_input_width);
                printf("CNN input height: %d\n", cnn_input_info.cnn_input_height);
                printf("CNN input num channels: %d\n", cnn_input_info.cnn_input_num_channels);

                // update tensor infos
                for (auto &ti : tensors_info)
                {
                    ti.nnet_input_width  = cnn_input_info.cnn_input_width;
                    ti.nnet_input_height = cnn_input_info.cnn_input_height;
                }

                c_streams_myriad_to_pc["previewout"].dimensions = {
                                                                cnn_input_info.cnn_input_num_channels,
                                                                cnn_input_info.cnn_input_height,
                                                                cnn_input_info.cnn_input_width
                                                                };

                // check CMX slices & used shaves
                int device_cmx_for_nnet = config_d2h.at("_resources").at("cmx").at("for_nnet").get<int>();
                if (cnn_input_info.number_of_cmx_slices != device_cmx_for_nnet)
                {
                    std::cout << "Error: Blob is compiled for " << cnn_input_info.number_of_cmx_slices
                            << " cmx slices but device can calculate on " << device_cmx_for_nnet << "\n";
                    break;
                }

                int device_shaves_for_nnet = config_d2h.at("_resources").at("shaves").at("for_nnet").get<int>();
                if (cnn_input_info.number_of_shaves != device_shaves_for_nnet)
                {
                    std::cout << "Error: Blob is compiled for " << cnn_input_info.number_of_shaves
                            << " shaves but device can calculate on " << device_shaves_for_nnet << "\n";
                    break;
                }
            }


            // sort streams by device specified order
            {
                // mapping: stream name -> array index
                std::vector<std::string> available_streams_ordered = getAvailableSteams();
                std::unordered_map<std::string, int> stream_name_to_idx;
                for (int i = 0; i < available_streams_ordered.size(); ++i)
                {
                    stream_name_to_idx[ available_streams_ordered[i] ] = i;
                }

                // check requested streams are in available streams
                bool wrong_stream_name = false;
                for (const auto &stream_name : pipeline_device_streams)
                {
                    if (stream_name_to_idx.find(stream_name) == stream_name_to_idx.end())
                    {
                        std::cout << "Error: device does not provide stream: " << stream_name << "\n";
                        wrong_stream_name = true;
                    }
                }

                if (wrong_stream_name)
                {
                    break;
                }

                // sort
                std::sort(std::begin(pipeline_device_streams), std::end(pipeline_device_streams),
                    [&stream_name_to_idx]
                    (const std::string &a, const std::string &b)
                    {
                        return stream_name_to_idx[a] < stream_name_to_idx[b];
                    }
                );
            }


            // pipeline
            pipeline.reset(new CNNHostPipeline(tensors_info));

            for (const std::string &stream_name : pipeline_device_streams)
            {
                std::cout << "Host stream start:" << stream_name << "\n";

                if (xlink->openStreamInThreadAndNotifyObservers(c_streams_myriad_to_pc.at(stream_name)))
                {
                    pipeline->makeStreamPublic(stream_name);
                    pipeline->observe(*xlink.get(), c_streams_myriad_to_pc.at(stream_name));
                }
                else
                {
                    std::cout << "depthai: " << stream_name << " error;\n";
                    // TODO: rollback correctly!
                    break;
                }
            }

            // disparity post processor
            if (add_disparity_post_processing_mm ||
                add_disparity_post_processing_color
            )
            {
                disparityPostProc.reset(
                    new DisparityStreamPostProcessor(
                        add_disparity_post_processing_color,
                        add_disparity_post_processing_mm
                    ));

                const std::string stream_in_name = "disparity";
                const std::string stream_out_color_name = "depth_color_h";
                const std::string stream_out_mm_name = "depth_mm_h";

                if (xlink->openStreamInThreadAndNotifyObservers(c_streams_myriad_to_pc.at(stream_in_name)))
                {
                    disparityPostProc->observe(*xlink.get(), c_streams_myriad_to_pc.at(stream_in_name));

                    if (add_disparity_post_processing_color)
                    {
                        pipeline->makeStreamPublic(stream_out_color_name);
                        pipeline->observe(*disparityPostProc.get(), c_streams_myriad_to_pc.at(stream_out_color_name));
                    }

                    if (add_disparity_post_processing_mm)
                    {
                        pipeline->makeStreamPublic(stream_out_mm_name);
                        pipeline->observe(*disparityPostProc.get(), c_streams_myriad_to_pc.at(stream_out_mm_name));
                    }
                }
                else
                {
                    std::cout << "depthai: stream open error " << stream_in_name << " (2)\n";
                    // TODO: rollback correctly!
                    break;
                }
            }

            if (!pipeline->setHostCalcDepthConfigs(
                    config.depth.type,
                    config.depth.padding_factor,
                    config.board_config.left_fov_deg,
                    config.board_config.left_to_right_distance_m
                    ))
            {
                std::cout << "depthai: Cant set depth;\n";
                break;
            }

            result = true;
            std::cout << "depthai: INIT OK!\n";
        }
        while (false);

        return result;
    }

    std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>
        > getAvailableNNetAndDataPackets()
    {
        return pipeline->getAvailableNNetAndDataPackets();
    }
};

// Constructor connected with our DepthAI_Impl structure 
DepthAI::DepthAI() 
    : pImpl(new DepthAI_Impl()) 
{ 
} 

DepthAI::~DepthAI() = default;
DepthAI::DepthAI(DepthAI&&) = default;
DepthAI& DepthAI::operator=(DepthAI&&) = default;

bool DepthAI::initDevice(
        const std::string &device_cmd_file
)
{ 
    return pImpl->initDevice(device_cmd_file); 
}

std::vector<std::string> DepthAI::getAvailableSteams()
{
    return pImpl->getAvailableSteams();
}

bool DepthAI::createPipeline(
    const std::string &config_json_str
)
{
    return pImpl->createPipeline(config_json_str);
}

std::tuple<
    std::list<std::shared_ptr<NNetPacket>>,
    std::list<std::shared_ptr<HostDataPacket>>
    > DepthAI::getAvailableNNetAndDataPackets()
{
    return pImpl->getAvailableNNetAndDataPackets();
}