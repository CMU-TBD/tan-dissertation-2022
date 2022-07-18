
#include "ipc_ros_bridge/ipc_bridge.h"
#include "ros/ros.h"

#include "geometry_msgs/Pose.h"

#include "ipc_ros_bridge/transformer/geometry_msgs_pose.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "geometry_relay_example");

    ros::NodeHandle nh;
    auto ipc = IPCBridge(nh, "ROS_IPC_Demo");
    std::cout << "Connected, Starting Relay ...." << std::endl;
    // Relay Message from IPC to ROS and publish on topic "fromIPC"
    ipc.RelayTopicFromIPC<geometry_msgs::Pose, GoemetryMsgsPose>("pose","pose");
    std::cout << "Starting Wait ...." << std::endl;
    ros::Duration wait(10);
    ipc.spin(wait);

    std::cout << "Stopping Relay ...." << std::endl;

    // Stop Mesage
    ipc.StopReceiveTopicFromIPC<std_msgs::String>("pose");
    ipc.Disconnect();

}
