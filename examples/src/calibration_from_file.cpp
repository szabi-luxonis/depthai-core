
#include <string>
#include <iostream>
#include <cstdio>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"


int main(){
    std::string filename("/home/sachin/Desktop/luxonis/depthai-core/examples/calib_data2.json");
    dai::CalibrationHandler calibData;
    
    calibData.setBoardInfo(6, true, "bw1098obc", "Rev");
    std::vector<std::vector<float>> inMatrix = {{1479.458984, 0.000000,    950.694458},
                                                {0.000000,    1477.587158, 530.697632},
                                                {0.000000,    0.000000,    1.000000  }};
    std::vector<float> inOneD = {-1.872860,   16.683033,    0.001053,   -0.002063,   61.878521,   -2.158907,   18.424637,
                57.682858,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000}; 
    calibData.setCameraIntrinsics(dai::CameraBoardSocket::RGB, inMatrix, 1920, 1080);
    calibData.setdistortionCoefficients(dai::CameraBoardSocket::RGB, inOneD);
    
    inMatrix = {{855.849548,    0.000000,  632.435974},
                {0.000000,    856.289001,  399.700226},
                {0.000000,      0.000000,    1.000000}};
    inOneD = {-4.481964,   14.138410,    0.002012,    0.000149,  -13.193083,   -4.541109,   14.358990,
                -13.394559,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000};
    
    calibData.setCameraIntrinsics(dai::CameraBoardSocket::LEFT, inMatrix, 1920, 1080);
    calibData.setdistortionCoefficients(dai::CameraBoardSocket::LEFT, inOneD);
    
    inMatrix = {{855.000122,  0.000000,  644.814514},
                {0.000000,  855.263794,  407.305450},
                {0.000000,    0.000000,    1.000000}};
    inOneD = {-5.598158,   19.114412,    0.000495,    0.000686,  -20.956785,   -5.645601,   19.298323,
                -21.132698,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000};
    
    calibData.setCameraIntrinsics(dai::CameraBoardSocket::RIGHT, inMatrix, 1920, 1080);
    calibData.setdistortionCoefficients(dai::CameraBoardSocket::RIGHT, inOneD);
    
    inMatrix = {{ 0.999903,    0.011196,    0.008257},
                {-0.011240,    0.999922,    0.005380},
                {-0.008196,   -0.005472,    0.999951}};
    inOneD = {-7.494308, 0.095795, 0.132222};
    calibData.setCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RIGHT, inMatrix, inOneD);
    
    inMatrix = {{ 0.999986,  0.004985,   0.001887},
                {-0.004995,  0.999974,   0.005245},
                {-0.001861, -0.005254,   0.999984}};
    inOneD = {3.782213, 0.002144, 0.122242}; 
    calibData.setCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::RGB, inMatrix, inOneD);
    

    inMatrix = {{0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}};
    inOneD = {0, 0, 0}; 
    calibData.setImuExtrinsics(dai::CameraBoardSocket::RGB, inMatrix, inOneD);

    inMatrix = {{0.999954,   -0.001489,   -0.009451},
                {0.001514,    0.999995,    0.002682},
                {0.009447,   -0.002697,    0.999952}};
    calibData.setStereoLeft(dai::CameraBoardSocket::LEFT, inMatrix);

    inMatrix = {{0.999763,   -0.012779,   -0.017639},
                {0.012732,    0.999915,   -0.002802},
                {0.017673,    0.002577,    0.999840}};
    calibData.setStereoRight(dai::CameraBoardSocket::RIGHT, inMatrix);
    calibData.eepromToJsonFile(filename);
    return 0;
    
}
