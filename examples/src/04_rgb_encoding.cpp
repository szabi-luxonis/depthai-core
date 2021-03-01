
#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(int argc, char** argv){

    using namespace std;
    using namespace std::chrono;

    std::string h265Path("video.h265");
    
    // If path specified, use that
    if(argc > 1){
        h265Path = std::string(argv[1]);
    }

    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xout = p.create<dai::node::XLinkOut>();
    auto xout2 = p.create<dai::node::XLinkOut>();
    auto videnc = p.create<dai::node::VideoEncoder>();

    // XLinkOut
    xout->setStreamName("h265");
    xout2->setStreamName("preview");

    // ColorCamera    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    //colorCam->setFps(5.0);
    colorCam->setInterleaved(true);

    // VideoEncoder
    videnc->setDefaultProfilePreset(3840, 2160, 30, dai::VideoEncoderProperties::Profile::H265_MAIN);

    // Link plugins CAM -> XLINK
    colorCam->video.link(videnc->input);
    colorCam->preview.link(xout2->input);
    videnc->bitstream.link(xout->input);

    // Connect to device with above created pipeline
    dai::Device d(p);
    // Start the pipeline
    d.startPipeline();

    auto myfile = std::fstream(h265Path, std::ios::out | std::ios::binary);


    auto h265Queue = d.getOutputQueue("h265", 8, false);
    auto previewQueue = d.getOutputQueue("preview", 8, false);
    while(1){

        auto preview = previewQueue->get<dai::ImgFrame>();
        cv::imshow("preview", cv::Mat(preview->getHeight(), preview->getWidth(), CV_8UC3, preview->getData().data()));
        auto h265 = h265Queue->get<dai::ImgFrame>();
        myfile.write((char*)h265->getData().data(), h265->getData().size());

        int key = cv::waitKey(1);
        if (key == 'q'){
            break;
        } 
    }
    myfile.close();

    std::cout << "To view the encoded data, convert the stream file " << h265Path << " into a video file (.mp4) using a command below:" << std::endl;
    std::cout << "ffmpeg -framerate " << colorCam->getFps() << " -i " << h265Path << " -c copy video.mp4" << std::endl;

    return 0;
}
