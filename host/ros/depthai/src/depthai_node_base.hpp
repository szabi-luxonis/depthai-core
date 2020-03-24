/*******************************************************************************
 *
 *
 *
 *******************************************************************************/

#ifndef __DEPTHAI_NODE_H__
#define __DEPTHAI_NODE_H__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <iomanip>
#include <depthai_node/CameraInfo.h>
#include <depthai_node/MetaOut.h>
#include <depthai_node/Entry.h>
#include <boost/smart_ptr.hpp>

#include <depthai.hpp>
#include <nnet/nnet_packet.hpp>
#include <host_data_packet.hpp>
#include <pipeline/cnn_host_pipeline.hpp>

using namespace std;

/**
 * \brief A driver node that receives data from depthai hw and forwards
 * it to ROS.
 */

namespace depthai_node {

class DepthAINodeBase {
public:
    ~DepthAINodeBase() {
    }

    /**
     * \brief Performs general initializations
     */
    void init();

    /*
     * \brief Collect and process packets
     */
    void processPackets();

	void createPipeline();

private:

    virtual ros::NodeHandle& getNH() = 0;
    virtual ros::NodeHandle& getPrivateNH() = 0;

    //
    boost::scoped_ptr<ros::Publisher> leftImagePublisher;
    boost::scoped_ptr<ros::Publisher> rightImagePublisher;
    boost::scoped_ptr<ros::Publisher> disparityImagePublisher;
    boost::scoped_ptr<ros::Publisher> previewOutPublisher;
    boost::scoped_ptr<ros::Publisher> depthmmHPublisher;
    boost::scoped_ptr<ros::Publisher> depthColorHPublisher;
    boost::scoped_ptr<ros::Publisher> cameraInfoPublisher;
    boost::scoped_ptr<ros::Publisher> metaOutPublisher;

	std::string configJson;

    // Parameters
    std::string cmdFile;
    std::string streams;
    std::string calibrationFile;
    std::string blobFile;
    std::string blobConfigFile;
    bool metaOut;
    bool previewOut;
    bool leftImage;
    bool rightImage;
    bool depthmmH;
    bool depthColorH;
    bool disparity;

    DepthAI depthai;

    // Other members
    int frameNum;
    
    depthai_node::CameraInfoPtr camInfoMsg;
    depthai_node::MetaOutPtr metaOutMsg;
    depthai_node::EntryPtr entryMsg;
    ros::Time lastCameraInfoPublish;

    ros::Time lastLogTime;
    int lastLogFrames = 0;

    /**
     *
     */
    void publishImageMsg(const std::shared_ptr<HostDataPacket> packet, ros::Time stamp, ros::Publisher* publisher);

    /**
     *
     */
    void publishMetaOutMsg(const std::vector<std::shared_ptr<HostDataPacket>> entries, ros::Time stamp);

    /**
     * \brief Publishes the camera info once per second
     */
    void publishCameraInfo(ros::Time stamp);

};

} // namespace

#endif

