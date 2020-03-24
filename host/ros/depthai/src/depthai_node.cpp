/*******************************************************************************
 * 
 *
 *
 *******************************************************************************/

#include "depthai_node_base.hpp"

namespace depthai_node {

class DepthAINode: public DepthAINodeBase {
public:
    DepthAINode(): privateNhInternal("~") { }

    /**
     * \brief The main loop of this node
     */
    int run() {
        ros::Rate loop_rate(60);
        createPipeline();
        try {
            while(ros::ok()) {
                // Dispatch any queued ROS callbacks
                ros::spinOnce();
                processPackets();
                loop_rate.sleep();
            }
        } catch(const std::exception& ex) {
            ROS_FATAL("Exception occured: %s", ex.what());
            return 1;
        }
        return 0;
    }
private:
    // The standalone node has its own private node handles
    ros::NodeHandle nhInternal;
    ros::NodeHandle privateNhInternal;
    inline ros::NodeHandle& getNH() override { return nhInternal; }
    inline ros::NodeHandle& getPrivateNH() override { return privateNhInternal; }
};

} // namespace

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "depthai");
        depthai_node::DepthAINode node;
        node.init();
        return node.run();
    } catch(const std::exception& ex) {
        ROS_FATAL("Exception occured: %s", ex.what());
        return 1;
    }
}

