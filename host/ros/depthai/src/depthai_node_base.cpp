/*******************************************************************************
 *
 *
 *******************************************************************************/

#include "depthai_node_base.hpp"

namespace depthai_node {

#define CHECK_ADD_SEPARATOR(s) do{if (!s.empty()) { s.append(", ");}}while(false)

/**
 * \brief Performs general initializations
 */
void DepthAINodeBase::init() {
    ros::NodeHandle& privateNh = getPrivateNH();

    // Read all ROS parameters
    if (!privateNh.getParam("cmd_file", cmdFile)) {
        cmdFile = "";
    }

    if (!privateNh.getParam("calibration_file", calibrationFile)) {
        calibrationFile = "";
    }

    if (!privateNh.getParam("blob_file", blobFile)) {
        blobFile = "";
    }

    if (!privateNh.getParam("blob_config_file", blobConfigFile)) {
        blobConfigFile = "";
    }

    if (!privateNh.getParam("metaout", metaOut)) {
        metaOut = false;
    }
    
    if (!privateNh.getParam("left", leftImage)) {
        leftImage = false;
    }

    if (!privateNh.getParam("right", rightImage)) {
        rightImage = false;
    }

    if (!privateNh.getParam("depth_mm_h", depthmmH)) {
        depthmmH = false;
    }

    if (!privateNh.getParam("depth_color_h", depthColorH)) {
        depthColorH = false;
    }

    if (!privateNh.getParam("disparity", disparity)) {
        disparity = false;
    }

    if (!privateNh.getParam("previewout", previewOut)) {
        previewOut = false;
    }

    if (metaOut) {
        streams.append("\"metaout\"");
    }

    if (previewOut) {
        CHECK_ADD_SEPARATOR(streams);
        streams.append("\"previewout\"");
    }

    if (leftImage) {
        CHECK_ADD_SEPARATOR(streams);
        streams.append("\"left\"");
    }

    if (rightImage) {
        CHECK_ADD_SEPARATOR(streams);
        streams.append("\"right\"");
    }

    if (depthmmH) {
        CHECK_ADD_SEPARATOR(streams);
        streams.append("\"depth_mm_h\"");
    }

    if (depthColorH) {
        CHECK_ADD_SEPARATOR(streams);
        streams.append("\"depth_color_h\"");
    }

    if (disparity) {
        CHECK_ADD_SEPARATOR(streams);
        streams.append("\"disparity\"");
    }

    if (streams.empty()) {
         streams.append(""); // check this
    }

    configJson = "{"
        "\"streams\": [" + streams + "],"
        "\"depth\":"
        "{"
            "\"calibration_file\": \"" + calibrationFile + "\","
            "\"padding_factor\": 0.3"
        "},"
        "\"ai\":"
        "{"
            "\"blob_file\": \"" + blobFile + "\","
            "\"blob_file_config\": \"" + blobConfigFile + "\","
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

    std::cout << "cmdFile = " << cmdFile << "\n";
    std::cout << "configJson = " << configJson << "\n";

    // Create publishers
    if (disparity){
        disparityImagePublisher.reset(new ros::Publisher(getNH().advertise<sensor_msgs::Image>(
            "/depthai_node/disparity", 512)));
    }
    if (leftImage) {
        leftImagePublisher.reset(new ros::Publisher(getNH().advertise<sensor_msgs::Image>(
            "/depthai_node/left_image", 512)));
    }

    if (rightImage) {
        rightImagePublisher.reset(new ros::Publisher(getNH().advertise<sensor_msgs::Image>(
            "/depthai_node/right_image", 512)));
    }

    if (previewOut) {
        previewOutPublisher.reset(new ros::Publisher(getNH().advertise<sensor_msgs::Image>(
            "/depthai_node/previewout", 512)));
    }

    if (depthmmH) {
        depthmmHPublisher.reset(new ros::Publisher(getNH().advertise<sensor_msgs::Image>(
            "/depthai_node/depth_mm_h", 512)));
    }

    if (depthColorH) {
        depthColorHPublisher.reset(new ros::Publisher(getNH().advertise<sensor_msgs::Image>(
            "/depthai_node/depth_color_h", 512)));
    }

    if (metaOut) {
        metaOutPublisher.reset(new ros::Publisher(getNH().advertise<depthai_node::MetaOut>(
            "/depthai_node/metaout", 512)));
    }

    // cameraInfoPublisher.reset(new ros::Publisher(getNH().advertise<depthai_node::CameraInfo>(
    //     "/depthai_node/camera_info", 5)));

    if(!depthai.initDevice(cmdFile)) {
        throw std::runtime_error("Failed to initialize device!");
    }
}

void DepthAINodeBase::createPipeline() {

    // std::cout << "streams = " ;   
    // std::vector<std::string> streams = depthai.getAvailableSteams();

    // for (std::vector<std::string>::const_iterator stream = streams.begin(); stream != streams.end(); ++stream)
    //     std::cout << *stream << ' ';

    // std::cout << "configJson = " << configJson << "\n";

    if(!depthai.createPipeline(configJson)) {
        throw std::runtime_error("Failed to create pipeline!");
    }
}

void DepthAINodeBase::processPackets() {

    std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>
        > packets = depthai.getAvailableNNetAndDataPackets();

    std::list<std::shared_ptr<NNetPacket>> nnet;
    std::list<std::shared_ptr<HostDataPacket>> data;
	tie(nnet, data) = packets;

    // Get time stamp
    ros::Time stamp;
    stamp = ros::Time::now();

    if (nnet.size() != 0) {
        for (const std::shared_ptr<NNetPacket> &packet : nnet) {
            std::vector<std::shared_ptr<HostDataPacket>> entries = packet->getTensors();
            publishMetaOutMsg(entries, stamp);       
		}

	}
	// std::cout << "HostDataPacket list size:" << data.size() << "\n";
    if (data.size() != 0){
        // std::cout << stamp << ": " << "packet list size:" << data.size() << "\n";
        for (const std::shared_ptr<HostDataPacket> &packet : data) {

            if (packet->stream_name == "left"){
                publishImageMsg(packet, stamp, leftImagePublisher.get());
			} else if (packet->stream_name == "right") {
                publishImageMsg(packet, stamp, rightImagePublisher.get());
            } else if (packet->stream_name == "disparity") {
                publishImageMsg(packet, stamp, disparityImagePublisher.get());
            } else if (packet->stream_name == "previewout") {
                publishImageMsg(packet, stamp, previewOutPublisher.get());
            } else if (packet->stream_name == "depth_mm_h") {
                publishImageMsg(packet, stamp, depthmmHPublisher.get());
            } else if (packet->stream_name == "depth_color_h") {
                publishImageMsg(packet, stamp, depthColorHPublisher.get());
            } else {

            }
		}
	}

    // if (cameraInfoPublisher != NULL && cameraInfoPublisher->getNumSubscribers() > 0) {
    //     publishCameraInfo(stamp);
    // }
}

void DepthAINodeBase::publishMetaOutMsg(const std::vector<std::shared_ptr<HostDataPacket>> entries, ros::Time stamp) {

    if (metaOutPublisher->getNumSubscribers() <= 0) {
        return; //No subscribers
    }

    metaOutMsg.reset(new depthai_node::MetaOut);
    entryMsg.reset(new depthai_node::Entry);

    for (int i = 0; i < entries.size(); ++i) {

        const uint8_t* data = entries[i]->getData();
        unsigned size = entries[i]->size();

        std::vector<uint8_t> vector(data, data + size);

        entryMsg->data = vector;

        metaOutMsg->entries.push_back(*entryMsg);
    }

    metaOutMsg->header.stamp = stamp;
    metaOutMsg->header.frame_id = entries[0]->stream_name;
    metaOutPublisher->publish(metaOutMsg);
}

void DepthAINodeBase::publishImageMsg(const std::shared_ptr<HostDataPacket> packet , ros::Time stamp, ros::Publisher* publisher) {

    if (publisher->getNumSubscribers() <= 0) {
        return; //No subscribers
    }
    bool ok = true;

    cv_bridge::CvImage cvImg;
    cvImg.header.frame_id = packet->stream_name;
    cvImg.header.stamp = stamp;
    cvImg.header.seq = 0; // Actually ROS will overwrite this
    string encoding = "";

    int rows = packet->dimensions[0];
    int cols = packet->dimensions[1];
    int size = packet->size();
    uint8_t *data =const_cast<uint8_t *>(packet->getData());

    // std::cout << "rows: " << rows << " cols: " << cols << " size:" << size << "\n";

    if (packet->stream_name == "left" ||
        packet->stream_name == "right" ||
        packet->stream_name == "disparity") {
        
        cv::Mat monoImg(rows, cols, CV_8UC1, data);

        cvImg.image = monoImg;
        encoding = "mono8";

    } else if (packet->stream_name == "previewout") {

        std::vector<cv::Mat> toMerge(3);
        cv::Mat bgrImg;
        int offset = cols*cols;
        cv::Mat ch1(cols, cols, CV_8UC1, data);
        cv::Mat ch2(cols, cols, CV_8UC1, data + offset);
        cv::Mat ch3(cols, cols, CV_8UC1, data + 2*offset);
        toMerge[0] = ch1;
        toMerge[1] = ch2;
        toMerge[2] = ch3;
        cv::merge(toMerge, bgrImg);
        cvImg.image = bgrImg;
        encoding = "bgr8";

        // Display some simple statistics
        frameNum++;
        if(stamp.sec != lastLogTime.sec) {
            if(lastLogTime != ros::Time()) {
                double dt = (stamp - lastLogTime).toSec();
                double fps = (frameNum - lastLogFrames) / dt;
                ROS_INFO("%.1f fps", fps);
            }
            lastLogFrames = frameNum;
            lastLogTime = stamp;
        }

    } else if (packet->stream_name.rfind("depth", 0) == 0 ){

        int ndim = packet->dimensions.size();
        int elemSize = packet->elem_size;
        if (ndim == 2) {
            if (elemSize == 1) {

                cv::Mat monoImg(rows, cols, CV_8UC1, data);
                cvImg.image = monoImg;
                encoding = "mono8";
            } else {

                cv::Mat bgrImg, monoImg;
                cv::Mat mono16Img(rows, cols, CV_16UC1, data);
                mono16Img = UINT16_MAX / mono16Img;
                mono16Img.convertTo(monoImg, CV_8UC1);
                cv::applyColorMap(monoImg, bgrImg, cv::COLORMAP_HOT);
                cvImg.image = bgrImg;
                encoding = "bgr8";
            }
        } else {

            cv::Mat bgrImg(rows, cols, CV_8UC3, reinterpret_cast<void *>(const_cast<uint8_t *>(packet->getData())));
            cvImg.image = bgrImg;
            encoding = "bgr8";
        }

    } else {
        ok = false;
    }

    if (ok) {
        sensor_msgs::ImagePtr msg = cvImg.toImageMsg();
        msg->encoding = encoding;
        publisher->publish(msg);
    }
}

// void DepthAINodeBase::publishCameraInfo(ros::Time stamp) {
//     if (cameraInfoPublisher->getNumSubscribers() <= 0) {
//         return; //No subscribers
//     }
//     if(camInfoMsg == NULL) {
//         // Initialize the camera info structure
//         camInfoMsg.reset(new depthai_node::CameraInfo);

//         camInfoMsg->header.frame_id = "";
//         camInfoMsg->header.seq = 0; // Actually ROS will overwrite this

//         camInfoMsg->camera_info.header = camInfoMsg->header;
//     }

//     double dt = (stamp - lastCameraInfoPublish).toSec();
//     if(dt > 1.0) {
//         // Publish once per second
//         camInfoMsg->header.stamp = stamp;
//         camInfoMsg->camera_info.header.stamp = stamp;
//         cameraInfoPublisher->publish(camInfoMsg);

//         lastCameraInfoPublish = stamp;
//     }
// }

} // namespace
