#ifndef __DEPTHAI_H__
#define __DEPTHAI_H__

#include <string>
#include <list>
#include <memory>

#include <nnet/nnet_packet.hpp>
#include <host_data_packet.hpp>
#include <pipeline/cnn_host_pipeline.hpp>

class DepthAI
{
public:

    ~DepthAI(); 
    DepthAI(); 


    DepthAI(DepthAI&&);
    DepthAI& operator=(DepthAI&&); 

    bool initDevice(
        const std::string &device_cmd_file
    );

    std::vector<std::string> getAvailableSteams();

    bool createPipeline(
        const std::string &config_json_str
    );

    std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>
        > getAvailableNNetAndDataPackets();

private: 
    // Internal implementation class 
    class DepthAI_Impl; 
  
    // Pointer to the internal implementation 
    std::unique_ptr<DepthAI_Impl> pImpl; 
};
#endif
