#include "xlink/XLinkConnection.hpp"

#include <XLink.h>

#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

extern "C" {
#include "XLinkLog.h"
}

namespace dai {

static DeviceInfo deviceInfoFix(const DeviceInfo& d, XLinkDeviceState_t state);

// STATIC
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_BOOTUP_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_CONNECT_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_STREAM_RETRY;
constexpr int XLinkConnection::STREAM_OPEN_RETRIES;

void XLinkConnection::initXLinkGlobal() {
    if(xlinkGlobalInitialized) return;

    xlinkGlobalHandler.protocol = X_LINK_USB_VSC;
    auto status = XLinkInitialize(&xlinkGlobalHandler);
    if(X_LINK_SUCCESS != status) {
        throw std::runtime_error("Couldn't initialize XLink");
    }

    // Suppress XLink related errors
    mvLogDefaultLevelSet(MVLOG_LAST);

    xlinkGlobalInitialized = true;
}

std::atomic<bool> XLinkConnection::xlinkGlobalInitialized{false};
XLinkGlobalHandler_t XLinkConnection::xlinkGlobalHandler = {};
std::mutex XLinkConnection::xlinkStreamOperationMutex;

std::vector<DeviceInfo> XLinkConnection::getAllConnectedDevices(XLinkDeviceState_t state) {
    initXLinkGlobal();

    std::vector<DeviceInfo> devices;

    std::vector<XLinkDeviceState_t> states;
    if(state == X_LINK_ANY_STATE) {
        states = {X_LINK_UNBOOTED,  X_LINK_BOOTED};
    } else {
        states = {state};
    }

    // Get all available devices (unbooted & booted)
    for(const auto& state : states) {
        unsigned int numdev = 0;
        std::array<deviceDesc_t, 32> deviceDescAll = {};
        deviceDesc_t suitableDevice = {};
        suitableDevice.protocol = X_LINK_ANY_PROTOCOL;
        suitableDevice.platform = X_LINK_ANY_PLATFORM;

        auto status = XLinkFindAllSuitableDevices(state, suitableDevice, deviceDescAll.data(), deviceDescAll.size(), &numdev);
        if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't retrieve all connected devices");

        for(unsigned i = 0; i < numdev; i++) {
            DeviceInfo info = {};
            info.desc = deviceDescAll.at(i);
            info.state = state;
            devices.push_back(info);
        }
    }

    return devices;
}

std::tuple<bool, DeviceInfo> XLinkConnection::getFirstDevice(XLinkDeviceState_t state) {
    auto devices = getAllConnectedDevices();
    for(const auto& d : devices) {
        if(d.state == state) return {true, d};
    }
    return {false, DeviceInfo()};
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary, XLinkDeviceState_t expectedState) {
    bootDevice = true;
    bootWithPath = false;
    this->mvcmd = std::move(mvcmdBinary);
    initDevice(deviceDesc, expectedState);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::string pathToMvcmd, XLinkDeviceState_t expectedState) {
    bootDevice = true;
    bootWithPath = true;
    this->pathToMvcmd = std::move(pathToMvcmd);
    initDevice(deviceDesc, expectedState);
}

// Skip boot
XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, XLinkDeviceState_t expectedState) {
    bootDevice = false;
    initDevice(deviceDesc, expectedState);
}

XLinkConnection::~XLinkConnection() {
    if(deviceLinkId != -1) {
        if(rebootOnDestruction) XLinkResetRemote(deviceLinkId);
    }
}

void XLinkConnection::setRebootOnDestruction(bool reboot) {
    rebootOnDestruction = reboot;
}

bool XLinkConnection::getRebootOnDestruction() const {
    return rebootOnDestruction;
}

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, const std::string& pathToMvcmd) {
    auto status = XLinkBoot((deviceDesc_t*)&deviceToBoot, pathToMvcmd.c_str());
    return status == X_LINK_SUCCESS;
}

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::vector<std::uint8_t>& mvcmd) {
    auto status = XLinkBootMemory((deviceDesc_t*)&deviceToBoot, mvcmd.data(), mvcmd.size());
    return status == X_LINK_SUCCESS;
}

void XLinkConnection::initDevice(const DeviceInfo& deviceToInit, XLinkDeviceState_t expectedState) {
    initXLinkGlobal();
    assert(deviceLinkId == -1);

    XLinkError_t rc = X_LINK_ERROR;
    deviceDesc_t deviceDesc = {};

    using namespace std::chrono;

    // if device is in UNBOOTED then boot
    bootDevice = deviceToInit.state == X_LINK_UNBOOTED;

    // boot device
    if(bootDevice) {
        DeviceInfo deviceToBoot = deviceInfoFix(deviceToInit, X_LINK_UNBOOTED);
        deviceDesc_t foundDeviceDesc = {};

        // Wait for the device to be available
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(X_LINK_UNBOOTED, deviceToBoot.desc, &foundDeviceDesc);
            if(rc == X_LINK_SUCCESS) break;
        } while(steady_clock::now() - tstart < WAIT_FOR_BOOTUP_TIMEOUT);

        if(bootWithPath) {
            bootAvailableDevice(foundDeviceDesc, pathToMvcmd);
        } else {
            bootAvailableDevice(foundDeviceDesc, mvcmd);
        }
    }

    // Search for booted device
    {
        // Create description of device to look for
        DeviceInfo bootedDeviceInfo = deviceInfoFix(deviceToInit, expectedState);

        // Find booted device
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(expectedState, bootedDeviceInfo.desc, &deviceDesc);
            if(rc == X_LINK_SUCCESS) break;
        } while(steady_clock::now() - tstart < WAIT_FOR_BOOTUP_TIMEOUT);

        if(rc != X_LINK_SUCCESS) {
            throw std::runtime_error("Failed to find device after booting, error message: " + convertErrorCodeToString(rc));
        }
    }

    // Try to connect to device
    {
        XLinkHandler_t connectionHandler = {};
        connectionHandler.devicePath = deviceDesc.name;
        connectionHandler.protocol = deviceDesc.protocol;

        auto tstart = steady_clock::now();
        do {
            if((rc = XLinkConnect(&connectionHandler)) == X_LINK_SUCCESS) break;
        } while(steady_clock::now() - tstart < WAIT_FOR_CONNECT_TIMEOUT);

        if(rc != X_LINK_SUCCESS) throw std::runtime_error("Failed to connect to device, error message: " + convertErrorCodeToString(rc));

        deviceLinkId = connectionHandler.linkId;
    }
}

void XLinkConnection::openStream(const std::string& streamName, std::size_t maxWriteSize) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");

    std::unique_lock<std::mutex> lock(xlinkStreamOperationMutex);
    streamId_t streamId = INVALID_STREAM_ID;

    assert(deviceLinkId != -1);

    for(int retryCount = 0; retryCount < STREAM_OPEN_RETRIES; retryCount++) {
        streamId = XLinkOpenStream(deviceLinkId, streamName.c_str(), maxWriteSize);

        if(streamId != INVALID_STREAM_ID) {
            break;
        }

        // Give some time before retrying
        std::this_thread::sleep_for(WAIT_FOR_STREAM_RETRY);
    }

    if(streamId == INVALID_STREAM_ID) throw std::runtime_error("Couldn't open stream");

    streamIdMap[streamName] = streamId;
}

void XLinkConnection::closeStream(const std::string& streamName) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");

    std::unique_lock<std::mutex> lock(xlinkStreamOperationMutex);
    if(streamIdMap.count(streamName) == 0) return;
    XLinkCloseStream(streamIdMap[streamName]);

    // remove from map
    streamIdMap.erase(streamName);
}

////////////////////
// BLOCKING VERSIONS
////////////////////

void XLinkConnection::writeToStream(const std::string& streamName, const std::uint8_t* data, std::size_t size) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");

    auto status = XLinkWriteData(streamIdMap[streamName], data, size);
    if(status != X_LINK_SUCCESS) throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
}
void XLinkConnection::writeToStream(const std::string& streamName, const void* data, std::size_t size) {
    writeToStream(streamName, reinterpret_cast<const uint8_t*>(data), size);
}

void XLinkConnection::writeToStream(const std::string& streamName, const std::vector<std::uint8_t>& data) {
    writeToStream(streamName, data.data(), data.size());
}

void XLinkConnection::readFromStream(const std::string& streamName, std::vector<std::uint8_t>& data) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");

    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadData(streamIdMap[streamName], &pPacket);
    if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't read data from stream: " + streamName);
    data = std::vector<std::uint8_t>(pPacket->data, pPacket->data + pPacket->length);
    XLinkReleaseData(streamIdMap[streamName]);
}

std::vector<std::uint8_t> XLinkConnection::readFromStream(const std::string& streamName) {
    std::vector<std::uint8_t> data;
    readFromStream(streamName, data);
    return data;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
streamPacketDesc_t* XLinkConnection::readFromStreamRaw(const std::string& streamName) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadData(streamIdMap[streamName], &pPacket);
    if(status != X_LINK_SUCCESS) {
        throw std::runtime_error("Error while reading data from xlink channel: " + streamName + " (" + XLinkErrorToStr(status) + ")");
    }
    return pPacket;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
void XLinkConnection::readFromStreamRawRelease(const std::string& streamName) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
    XLinkReleaseData(streamIdMap[streamName]);
}

// SPLIT HELPER
void XLinkConnection::writeToStreamSplit(const std::string& streamName, const void* d, std::size_t size, std::size_t split) {
    const uint8_t* data = (const uint8_t*)d;
    std::size_t currentOffset = 0;
    std::size_t remaining = size;
    std::size_t sizeToTransmit = 0;
    XLinkError_t ret = X_LINK_SUCCESS;
    streamId_t streamId = getStreamId(streamName);
    while(remaining > 0) {
        sizeToTransmit = remaining > split ? split : remaining;
        ret = XLinkWriteData(streamId, data + currentOffset, sizeToTransmit);
        if(ret != X_LINK_SUCCESS) throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(ret));
        currentOffset += sizeToTransmit;
        remaining = size - currentOffset;
    }
}

void XLinkConnection::writeToStreamSplit(const std::string& streamName, const std::vector<uint8_t>& data, std::size_t split) {
    writeToStreamSplit(streamName, data.data(), data.size(), split);
}

///////////////////////
// Timeout versions //
//////////////////////

// bool XLinkConnection::writeToStream(const std::string& streamName, const std::uint8_t* data, std::size_t size, std::chrono::milliseconds timeout) {
//     if(streamName.empty()) throw std::invalid_argument("streamName is empty");
//     if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
//     auto status = XLinkWriteDataWithTimeout(streamIdMap[streamName], data, size, timeout.count());
//     if(status == X_LINK_SUCCESS) return true;
//     else if(status == X_LINK_TIMEOUT) return false;
//     else throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
// }
//
// bool XLinkConnection::writeToStream(const std::string& streamName, const void* data, std::size_t size, std::chrono::milliseconds timeout) {
//     return writeToStream(streamName, reinterpret_cast<const std::uint8_t*>(data), size, timeout);
// }
//
// bool XLinkConnection::writeToStream(const std::string& streamName, const std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout) {
//     return writeToStream(streamName, data.data(), data.size(), timeout);
// }
//
// bool XLinkConnection::readFromStream(const std::string& streamName, std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout) {
//     if(streamName.empty()) throw std::invalid_argument("streamName is empty");
//     if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
//
//     streamPacketDesc_t* pPacket = nullptr;
//     auto status = XLinkReadDataWithTimeout(streamIdMap[streamName], &pPacket, timeout.count());
//
//     if(status == X_LINK_SUCCESS){
//         data = std::vector<std::uint8_t>(pPacket->data, pPacket->data + pPacket->length);
//         XLinkReleaseData(streamIdMap[streamName]);
//         return true;
//     } else if(status == X_LINK_TIMEOUT){
//         return false;
//     } else {
//         throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
//     }
//
//     return false;
// }
//
// bool XLinkConnection::readFromStreamRaw(streamPacketDesc_t*& pPacket, const std::string& streamName, std::chrono::milliseconds timeout) {
//     if(streamName.empty()) throw std::invalid_argument("streamName is empty");
//     if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
//     auto status = XLinkReadDataWithTimeout(streamIdMap[streamName], &pPacket, timeout.count());
//     if(status == X_LINK_SUCCESS) return true;
//     else if(status == X_LINK_TIMEOUT) return false;
//     else throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
// }

streamId_t XLinkConnection::getStreamId(const std::string& name) const {
    return streamIdMap.at(name);
}

int XLinkConnection::getLinkId() const {
    return deviceLinkId;
}

std::string XLinkConnection::convertErrorCodeToString(XLinkError_t errorCode) {
    switch(errorCode) {
        case X_LINK_SUCCESS:
            return "X_LINK_SUCCESS";
        case X_LINK_ALREADY_OPEN:
            return "X_LINK_ALREADY_OPEN";
        case X_LINK_COMMUNICATION_NOT_OPEN:
            return "X_LINK_COMMUNICATION_NOT_OPEN";
        case X_LINK_COMMUNICATION_FAIL:
            return "X_LINK_COMMUNICATION_FAIL";
        case X_LINK_COMMUNICATION_UNKNOWN_ERROR:
            return "X_LINK_COMMUNICATION_UNKNOWN_ERROR";
        case X_LINK_DEVICE_NOT_FOUND:
            return "X_LINK_DEVICE_NOT_FOUND";
        case X_LINK_TIMEOUT:
            return "X_LINK_TIMEOUT";
        case X_LINK_ERROR:
            return "X_LINK_ERROR";
        case X_LINK_OUT_OF_MEMORY:
            return "X_LINK_OUT_OF_MEMORY";
        default:
            return "<UNKNOWN ERROR>";
    }
}

DeviceInfo deviceInfoFix(const DeviceInfo& dev, XLinkDeviceState_t state) {
    DeviceInfo fixed(dev);

    // Remove everything after dash
    for(int i = sizeof(fixed.desc.name) - 1; i >= 0; i--) {
        if(fixed.desc.name[i] != '-')
            fixed.desc.name[i] = 0;
        else
            break;
    }

    if(state == X_LINK_UNBOOTED) {
        // if unbooted add ending ("ma2480" or "ma2450")
        if(fixed.desc.platform == X_LINK_MYRIAD_2) {
            std::strncat(fixed.desc.name, "ma2450", sizeof(fixed.desc.name) - std::strlen(fixed.desc.name));
        } else {
            std::strncat(fixed.desc.name, "ma2480", sizeof(fixed.desc.name) - std::strlen(fixed.desc.name));
        }
    } else if(state == X_LINK_BOOTED) {
        // set platform to any
        fixed.desc.platform = X_LINK_ANY_PLATFORM;
    }

    return fixed;
}

}  // namespace dai
