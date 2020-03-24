#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <exception>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include <depthai/depthai.hpp>
#include <depthai/nnet/nnet_packet.hpp>
#include <depthai/host_data_packet.hpp>
#include <depthai/pipeline/cnn_host_pipeline.hpp>

#define DEPTHAI_EXTRAS_PATH "../../../../../../"

const char* cmd_file = DEPTHAI_EXTRAS_PATH "depthai.cmd";

std::string calibration_file = DEPTHAI_EXTRAS_PATH "resources/default.calib";

std::string blob_file = DEPTHAI_EXTRAS_PATH "resources/nn/object_detection_4shave/mobilenet_ssd.blob";
std::string blob_config_file = DEPTHAI_EXTRAS_PATH "resources/nn/object_detection_4shave/object_detection.json";

std::string streams = "\"metaout\", \"left\"";

int break_counter = 0;
bool do_break = false;

DepthAI depthai;


void breakHandler(int s)
{
    printf("depthai: caught signal %d. exiting...\n", s);

    do_break = true;

    if (break_counter != 0) { exit(1); }
    ++break_counter;
}


int main(int argc, char** argv)
try
{
    std::cout << "depthai: start\n";

    // handler for Ctrl+C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = breakHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    std::string config_json = "{"
        "\"streams\": [" + streams + "],"
        "\"depth\":"
        "{"
            "\"calibration_file\": \"" + calibration_file + "\","
            "\"padding_factor\": 0.3"
        "},"
        "\"ai\":"
        "{"
            "\"blob_file\": \"" + blob_file + "\","
            "\"blob_file_config\": \"" + blob_config_file + "\","
            "\"calc_dist_to_bb\": true"
        "},"
        "\"board_config\":"
        "{"
            "\"swap_left_and_right_cameras\": false,"
            "\"left_fov_deg\": 69.0,"
            "\"left_to_right_distance_cm\": 9.0,"
            "\"left_to_rgb_distance_cm\": 2.0"
        "}"
    "}";

    if(!depthai.initDevice(cmd_file)) {
        throw std::runtime_error("failed to initialize device!");
    }

    if(!depthai.createPipeline(config_json)) {
        throw std::runtime_error("failed to create pipeline!");
    }

    auto start = std::chrono::steady_clock::now();

    while (true) {

        std::tuple<
            std::list<std::shared_ptr<NNetPacket>>,
            std::list<std::shared_ptr<HostDataPacket>>
            > packets = depthai.getAvailableNNetAndDataPackets();

        std::list<std::shared_ptr<NNetPacket>> nnet;
        std::list<std::shared_ptr<HostDataPacket>> data;
	    tie(nnet, data) = packets;

        if (data.size() != 0){
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
            std::cout << elapsed.count() << ": " << "packet list size: " << data.size() << "\n";
        }

        if (do_break){
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "depthai: done\n";
    return EXIT_SUCCESS;

}

catch (const std::exception &e)
{
    std::cout << "depthai: std exception: " << e.what() << "\n";
}
catch (...)
{
    std::cout << "depthai: global exception.\n";
}
