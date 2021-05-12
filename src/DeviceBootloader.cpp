#include "DeviceBootloader.hpp"

// std
#include <fstream>

// shared
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/SBR.h"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai-shared/Assets.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "BootloaderHelper.hpp"
#include "Device.hpp"
#include "pipeline/Pipeline.hpp"

// Resource compiled assets (cmds)
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES
    #include "cmrc/cmrc.hpp"
CMRC_DECLARE(depthai);
#endif

namespace dai {

// static api

// First tries to find UNBOOTED device, then BOOTLOADER device
std::tuple<bool, DeviceInfo> DeviceBootloader::getFirstAvailableDevice() {
    bool found;
    DeviceInfo dev;
    std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_UNBOOTED);
    if(!found) {
        assert(0);
    }
    return {found, dev};
}

// Returns all devices which aren't already booted
std::vector<DeviceInfo> DeviceBootloader::getAllAvailableDevices() {
    std::vector<DeviceInfo> availableDevices;
    auto connectedDevices = XLinkConnection::getAllConnectedDevices();
    for(const auto& d : connectedDevices) {
        if(d.state != X_LINK_BOOTED) availableDevices.push_back(d);
    }
    return availableDevices;
}

std::vector<uint8_t> DeviceBootloader::createDepthaiApplicationPackage(Pipeline& pipeline, std::string pathToCmd) {
    // Prepare device firmware
    std::vector<uint8_t> deviceFirmware;
    if(pathToCmd != "") {
        std::ifstream fwStream(pathToCmd, std::ios::in | std::ios::binary);
        if(!fwStream.is_open()) throw std::runtime_error("Cannot create application package, device firmware at path: " + pathToCmd + " doesn't exist");
        deviceFirmware = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
    } else {
        deviceFirmware = Device::getEmbeddedDeviceBinary(false);
    }

    // Serialize the pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    pipeline.serialize(schema, assets, assetStorage);

    std::vector<uint8_t> pipelineBinary, assetsBinary;

    // Create msgpacks
    {
        nlohmann::json j = schema;
        pipelineBinary = nlohmann::json::to_msgpack(j);
    }
    {
        nlohmann::json j = assets;
        assetsBinary = nlohmann::json::to_msgpack(j);
    }

    // Prepare SBR structure
    SBR sbr = {};
    SBR_SECTION* fwSection = &sbr.sections[0];
    SBR_SECTION* pipelineSection = &sbr.sections[1];
    SBR_SECTION* assetsSection = &sbr.sections[2];
    SBR_SECTION* assetStorageSection = &sbr.sections[3];
    SBR_SECTION* lastSection = assetStorageSection;

    // Alignup for easier updating
    auto getSectionAlignedOffset = [](long S) {
        constexpr long SECTION_ALIGNMENT_SIZE = 1 * 1024 * 1024;  // 1MiB for easier updating
        return ((((S) + (SECTION_ALIGNMENT_SIZE)-1)) & ~((SECTION_ALIGNMENT_SIZE)-1));
    };

    // First section, MVCMD, name '__firmware'
    sbr_section_set_name(fwSection, "__firmware");
    sbr_section_set_bootable(fwSection, true);
    sbr_section_set_size(fwSection, deviceFirmware.size());
    sbr_section_set_checksum(fwSection, sbr_compute_checksum(deviceFirmware.data(), deviceFirmware.size()));
    sbr_section_set_offset(fwSection, SBR_RAW_SIZE);

    // Second section, pipeline schema, name 'pipeline'
    sbr_section_set_name(pipelineSection, "pipeline");
    sbr_section_set_size(pipelineSection, pipelineBinary.size());
    sbr_section_set_checksum(pipelineSection, sbr_compute_checksum(pipelineBinary.data(), pipelineBinary.size()));
    sbr_section_set_offset(pipelineSection, getSectionAlignedOffset(fwSection->offset + fwSection->size));

    // Third section, assets map, name 'assets'
    sbr_section_set_name(assetsSection, "assets");
    sbr_section_set_size(assetsSection, assetsBinary.size());
    sbr_section_set_checksum(assetsSection, sbr_compute_checksum(assetsBinary.data(), assetsBinary.size()));
    sbr_section_set_offset(assetsSection, getSectionAlignedOffset(pipelineSection->offset + pipelineSection->size));

    // Fourth section, asset storage, name 'asset_storage'
    sbr_section_set_name(assetStorageSection, "asset_storage");
    sbr_section_set_size(assetStorageSection, assetStorage.size());
    sbr_section_set_checksum(assetStorageSection, sbr_compute_checksum(assetStorage.data(), assetStorage.size()));
    sbr_section_set_offset(assetStorageSection, getSectionAlignedOffset(assetsSection->offset + assetsSection->size));

    // TODO(themarpe) - Add additional sections (Pipeline nodes will be able to use sections)

    // Create a vector to hold whole dap package
    std::vector<uint8_t> fwPackage;
    fwPackage.resize(lastSection->offset + lastSection->size);

    // Serialize SBR
    sbr_serialize(&sbr, fwPackage.data(), fwPackage.size());

    // Write to fwPackage
    for(unsigned i = 0; i < deviceFirmware.size(); i++) fwPackage[fwSection->offset + i] = deviceFirmware[i];
    for(unsigned i = 0; i < pipelineBinary.size(); i++) fwPackage[pipelineSection->offset + i] = pipelineBinary[i];
    for(unsigned i = 0; i < assetsBinary.size(); i++) fwPackage[assetsSection->offset + i] = assetsBinary[i];
    for(unsigned i = 0; i < assetStorage.size(); i++) fwPackage[assetStorageSection->offset + i] = assetStorage[i];

    return fwPackage;
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo) : deviceInfo(devInfo) {
    init(true, "");
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, const char* pathToBootloader) : deviceInfo(devInfo) {
    init(false, std::string(pathToBootloader));
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, const std::string& pathToBootloader) : deviceInfo(devInfo) {
    init(false, pathToBootloader);
}

DeviceBootloader::~DeviceBootloader() {
    // Stop watchdog first
    watchdogRunning = false;
    if(watchdogThread.joinable()) watchdogThread.join();
}

void DeviceBootloader::init(bool embeddedMvcmd, const std::string& pathToMvcmd) {
    assert(0);

}

DeviceBootloader::Version DeviceBootloader::getEmbeddedBootloaderVersion() {
    return DeviceBootloader::Version(DEPTHAI_BOOTLOADER_VERSION);
}

DeviceBootloader::Version DeviceBootloader::getVersion() {
    streamId_t streamId = connection->getStreamId(bootloader::XLINK_CHANNEL_BOOTLOADER);

    // Send request to jump to USB bootloader
    if(!sendBootloaderRequest(streamId, bootloader::request::GetBootloaderVersion{})) {
        throw std::runtime_error("Couldn't get bootloader version");
    }

    // Receive response
    dai::bootloader::response::BootloaderVersion ver;
    if(!receiveBootloaderResponse(streamId, ver)) {
        throw std::runtime_error("Couldn't get bootloader version");
    }

    // Create bootloader::Version object and return
    return DeviceBootloader::Version(ver.major, ver.minor, ver.patch);
}

std::tuple<bool, std::string> DeviceBootloader::flash(std::function<void(float)> progressCb, Pipeline& pipeline) {
    return flashDepthaiApplicationPackage(progressCb, createDepthaiApplicationPackage(pipeline));
}

std::tuple<bool, std::string> DeviceBootloader::flashDepthaiApplicationPackage(std::function<void(float)> progressCb, std::vector<uint8_t> package) {
    streamId_t streamId = connection->getStreamId(bootloader::XLINK_CHANNEL_BOOTLOADER);

    // send request to FLASH BOOTLOADER
    dai::bootloader::request::UpdateFlash updateFlash;
    updateFlash.storage = dai::bootloader::request::UpdateFlash::SBR;
    updateFlash.totalSize = package.size();
    updateFlash.numPackets = ((package.size() - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendBootloaderRequest(streamId, updateFlash)) return {false, "Couldn't send bootloader flash request"};

    // After that send numPackets of data
    connection->writeToStreamSplit(bootloader::XLINK_CHANNEL_BOOTLOADER, package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    dai::bootloader::response::FlashComplete result;
    do {
        std::vector<uint8_t> data;
        if(!receiveBootloaderResponseData(streamId, data)) return {false, "Couldn't receive bootloader response"};

        dai::bootloader::response::FlashStatusUpdate update;
        if(parseBootloaderResponse(data, update)) {
            // if progress callback is set
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
        } else if(parseBootloaderResponse(data, result)) {
            break;
        } else {
            // Unknown response, shouldn't happen
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashBootloader(std::function<void(float)> progressCb, std::string path) {
    std::vector<uint8_t> package;
    if(path != "") {
        std::ifstream fwStream(path, std::ios::in | std::ios::binary);
        if(!fwStream.is_open()) throw std::runtime_error("Cannot flash bootloader, binary at path: " + path + " doesn't exist");
        package = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
    } else {
        package = getEmbeddedBootloaderBinary();
    }

    // get streamId
    streamId_t streamId = connection->getStreamId(bootloader::XLINK_CHANNEL_BOOTLOADER);

    // send request to FLASH BOOTLOADER
    dai::bootloader::request::UpdateFlash updateFlash;
    updateFlash.storage = dai::bootloader::request::UpdateFlash::BOOTLOADER;
    updateFlash.totalSize = package.size();
    updateFlash.numPackets = ((package.size() - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendBootloaderRequest(streamId, updateFlash)) return {false, "Couldn't send bootloader flash request"};

    // After that send numPackets of data
    connection->writeToStreamSplit(bootloader::XLINK_CHANNEL_BOOTLOADER, package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    dai::bootloader::response::FlashComplete result;
    do {
        std::vector<uint8_t> data;
        if(!receiveBootloaderResponseData(streamId, data)) return {false, "Couldn't receive bootloader response"};

        dai::bootloader::response::FlashStatusUpdate update;
        if(parseBootloaderResponse(data, update)) {
            // if progress callback is set
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
            // if flash complete response arrived, break from while loop
        } else if(parseBootloaderResponse(data, result)) {
            break;
        } else {
            // Unknown response, shouldn't happen
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}

std::vector<std::uint8_t> DeviceBootloader::getEmbeddedBootloaderBinary() {
// Binaries are resource compiled
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    constexpr static auto CMRC_DEPTHAI_BOOTLOADER_PATH = "depthai-bootloader-" DEPTHAI_BOOTLOADER_VERSION ".cmd";

    // Get binaries from internal sources
    auto fs = cmrc::depthai::get_filesystem();

    auto bootloaderBinary = fs.open(CMRC_DEPTHAI_BOOTLOADER_PATH);
    return std::vector<std::uint8_t>(bootloaderBinary.begin(), bootloaderBinary.end());

#else
    assert(0 && "Unsupported");
    return {};
#endif
}

bool DeviceBootloader::isEmbeddedVersion() {
    return isEmbedded;
}

DeviceBootloader::Version::Version(const std::string& v) : versionMajor(0), versionMinor(0), versionPatch(0) {
    // Parse string
    if(std::sscanf(v.c_str(), "%u.%u.%u", &versionMajor, &versionMinor, &versionPatch) != 3) throw std::runtime_error("Cannot parse version: " + v);
}

DeviceBootloader::Version::Version(unsigned vmajor, unsigned vminor, unsigned vpatch) {
    this->versionMajor = vmajor;
    this->versionMinor = vminor;
    this->versionPatch = vpatch;
}

bool DeviceBootloader::Version::operator==(const Version& other) const {
    if(versionMajor == other.versionMajor && versionMinor == other.versionMinor && versionPatch == other.versionPatch) return true;
    return false;
}

bool DeviceBootloader::Version::operator>(const Version& other) const {
    if(versionMajor > other.versionMajor) {
        return true;
    } else {
        if(versionMinor > other.versionMinor) {
            return true;
        } else {
            if(versionPatch > other.versionPatch) {
                return true;
            }
        }
    }
    return false;
}

bool DeviceBootloader::Version::operator<(const Version& other) const {
    if(versionMajor < other.versionMajor) {
        return true;
    } else {
        if(versionMinor < other.versionMinor) {
            return true;
        } else {
            if(versionPatch < other.versionPatch) {
                return true;
            }
        }
    }
    return false;
}

std::string DeviceBootloader::Version::toString() const {
    return std::to_string(versionMajor) + "." + std::to_string(versionMinor) + "." + std::to_string(versionPatch);
}

}  // namespace dai
