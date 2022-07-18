
#include "ipc_ros_bridge/ipc_bridge.h"
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "ipc_ros_bridge/transformer/std_msgs_string.h"



void receiveMsg(std_msgs::String msg){
    std::cout << "Received:" << msg.data << std::endl;
}

void receiveMsgBindArg(std_msgs::String msg, std::string info){
    std::cout << "Received:" << msg.data << "with" << info << std::endl;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "receive_node");
    ros::NodeHandle nh;
    auto ipc = IPCBridge(nh, "ros_ipc_receive_example");

    ipc.ReceiveTopicFromIPC<std_msgs::String, StdMsgsString>("ROSMSG1", receiveMsg);

    std::cout << "Starting Listening ...." << std::endl;
    ros::Duration wait(10);
    ipc.spin(wait);

    std::cout << "Stopping Listening ...." << std::endl;

    ipc.StopReceiveTopicFromIPC<std_msgs::String>("ROSMSG1");

    ipc.spin(ros::Duration(5));

    std::cout << "Restarting Listening ...." << std::endl;

    // Bind examples to pass in variables or other information to the callback function
    auto cb = std::bind(receiveMsgBindArg, std::placeholders::_1, "info");
    ipc.ReceiveTopicFromIPC<std_msgs::String, StdMsgsString>("ROSMSG1", cb);

    ipc.spin(wait);
    
    std::cout << "Stopping Listening ...." << std::endl;


    ipc.StopReceiveTopicFromIPC<std_msgs::String>("ROSMSG1");

    ipc.Disconnect();
}
