
#include "ipc_ros_bridge/ipc_bridge.h"
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "ipc_ros_bridge/transformer/std_msgs_string.h"
#include "ipc_ros_bridge/transformer/std_msgs_bool.h"
#include "ipc_ros_bridge/transformer/std_msgs_float64.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "my_node_name");

    ros::NodeHandle nh;
    auto ipc = IPCBridge(nh, "ROS_IPC_Demo");
    std::cout << "Connected, Starting Relay ...." << std::endl;
    // Relay Message from IPC to ROS and publish on topic "fromIPC"
    ipc.RelayTopicFromIPC<std_msgs::String, StdMsgsString>("ROSMSG1","fromIPC");
    // Relay the Message from topic "t" to IPC
    ipc.RelayTopicToIPC<std_msgs::String, StdMsgsString>("IPCMSG1","t"); 
    ipc.RelayTopicToIPC<std_msgs::Float64, StdMsgsFloat64>("IPCMSG2","f"); 

    std::cout << "Starting Wait ...." << std::endl;
    ros::Duration wait(10);
    ipc.spin(wait);

    std::cout << "Stopping Relay ...." << std::endl;

    // Stop Mesage
    ipc.StopReceiveTopicFromIPC<std_msgs::String>("fromIPC");
    ipc.StopRelayTopicToIPC<std_msgs::String>("t"); 
    ipc.StopRelayTopicToIPC<std_msgs::String>("f"); 

    ipc.Disconnect();

}
