#include "depthai/xlink/XLinkStream.hpp"

// libraries
#include "XLink/XLink.h"
#include "spdlog/fmt/fmt.h"

// project
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

// static
constexpr std::chrono::milliseconds XLinkStream::WAIT_FOR_STREAM_RETRY;
constexpr int XLinkStream::STREAM_OPEN_RETRIES;
std::mutex XLinkStream::xlinkStreamOperationMutex;

XLinkStream::XLinkStream(const XLinkConnection& conn, const std::string& name, std::size_t maxWriteSize) : streamName(name) {
    if(name.empty()) throw std::invalid_argument("Cannot create XLinkStream using empty stream name");
    if(conn.getLinkId() == -1) throw std::invalid_argument("Cannot create XLinkStream using unconnected XLinkConnection");

    std::unique_lock<std::mutex> lock(xlinkStreamOperationMutex);
    streamId = INVALID_STREAM_ID;

    for(int retryCount = 0; retryCount < STREAM_OPEN_RETRIES; retryCount++) {
        streamId = XLinkOpenStream(conn.getLinkId(), streamName.c_str(), static_cast<int>(maxWriteSize));

        // Give some time before continuing
        std::this_thread::sleep_for(WAIT_FOR_STREAM_RETRY);

        if(streamId != INVALID_STREAM_ID) {
            break;
        }
    }

    if(streamId == INVALID_STREAM_ID) throw std::runtime_error("Couldn't open stream");
}

// Move constructor
XLinkStream::XLinkStream(XLinkStream&& stream) : streamName(std::move(stream.streamName)) {
    // Construct from 'stream' into current
    // Just copy its streamId
    streamId = stream.streamId;

    // Set 'stream's streamId to INVALID_STREAM_ID to prevent closing
    stream.streamId = INVALID_STREAM_ID;
}

XLinkStream::~XLinkStream() {
    // If streamId != invalid (eg. wasn't moved to another XLinkStream)
    if(streamId != INVALID_STREAM_ID) {
        std::unique_lock<std::mutex> lock(xlinkStreamOperationMutex);
        XLinkCloseStream(streamId);
        std::this_thread::sleep_for(WAIT_FOR_STREAM_RETRY);
    }
}

////////////////////
// BLOCKING VERSIONS
////////////////////

void XLinkStream::write(const std::uint8_t* data, std::size_t size) {
    auto status = XLinkWriteData(streamId, data, static_cast<int>(size));
    if(status != X_LINK_SUCCESS) {
        throw std::runtime_error(fmt::format("Couldn't write data to stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(status)));
    }
}
void XLinkStream::write(const void* data, std::size_t size) {
    write(reinterpret_cast<const uint8_t*>(data), size);
}

void XLinkStream::write(const std::vector<std::uint8_t>& data) {
    write(data.data(), data.size());
}

void XLinkStream::read(std::vector<std::uint8_t>& data) {
    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadData(streamId, &pPacket);
    if(status != X_LINK_SUCCESS) {
        throw std::runtime_error(fmt::format("Couldn't read data from stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(status)));
    }
    data = std::vector<std::uint8_t>(pPacket->data, pPacket->data + pPacket->length);
    XLinkReleaseData(streamId);
}

std::vector<std::uint8_t> XLinkStream::read() {
    std::vector<std::uint8_t> data;
    read(data);
    return data;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
streamPacketDesc_t* XLinkStream::readRaw() {
    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadData(streamId, &pPacket);
    if(status != X_LINK_SUCCESS) {
        throw std::runtime_error(fmt::format("Couldn't read data from stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(status)));
    }
    return pPacket;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
void XLinkStream::readRawRelease() {
    XLinkReleaseData(streamId);
}

// SPLIT HELPER
void XLinkStream::writeSplit(const void* d, std::size_t size, std::size_t split) {
    const uint8_t* data = (const uint8_t*)d;
    std::size_t currentOffset = 0;
    std::size_t remaining = size;
    std::size_t sizeToTransmit = 0;
    XLinkError_t ret = X_LINK_SUCCESS;
    while(remaining > 0) {
        sizeToTransmit = remaining > split ? split : remaining;
        ret = XLinkWriteData(streamId, data + currentOffset, static_cast<int>(sizeToTransmit));
        if(ret != X_LINK_SUCCESS) {
            throw std::runtime_error(fmt::format("Couldn't write data to stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(ret)));
        }
        currentOffset += sizeToTransmit;
        remaining = size - currentOffset;
    }
}

void XLinkStream::writeSplit(const std::vector<uint8_t>& data, std::size_t split) {
    writeSplit(data.data(), data.size(), split);
}

///////////////////////
// Timeout versions //
//////////////////////

bool XLinkStream::write(const std::uint8_t* data, std::size_t size, std::chrono::milliseconds timeout) {
    auto status = XLinkWriteDataWithTimeout(streamId, data, static_cast<int>(size), static_cast<unsigned int>(timeout.count()));
    if(status == X_LINK_SUCCESS) {
        return true;
    } else if(status == X_LINK_TIMEOUT) {
        return false;
    } else {
        throw std::runtime_error(fmt::format("Couldn't write data to stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(status)));
    }
}

bool XLinkStream::write(const void* data, std::size_t size, std::chrono::milliseconds timeout) {
    return write(reinterpret_cast<const std::uint8_t*>(data), size, timeout);
}

bool XLinkStream::write(const std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout) {
    return write(data.data(), data.size(), timeout);
}

bool XLinkStream::read(std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout) {
    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadDataWithTimeout(streamId, &pPacket, static_cast<unsigned int>(timeout.count()));
    if(status == X_LINK_SUCCESS) {
        data = std::vector<std::uint8_t>(pPacket->data, pPacket->data + pPacket->length);
        XLinkReleaseData(streamId);
        return true;
    } else if(status == X_LINK_TIMEOUT) {
        return false;
    } else {
        throw std::runtime_error(fmt::format("Couldn't read data from stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(status)));
    }
    return false;
}

bool XLinkStream::readRaw(streamPacketDesc_t*& pPacket, std::chrono::milliseconds timeout) {
    auto status = XLinkReadDataWithTimeout(streamId, &pPacket, static_cast<unsigned int>(timeout.count()));
    if(status == X_LINK_SUCCESS) {
        return true;
    } else if(status == X_LINK_TIMEOUT) {
        return false;
    } else {
        throw std::runtime_error(fmt::format("Couldn't read data from stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(status)));
    }
}

streamId_t XLinkStream::getStreamId() const {
    return streamId;
}

}  // namespace dai
