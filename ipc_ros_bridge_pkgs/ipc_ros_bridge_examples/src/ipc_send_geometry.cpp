#include <iostream>
#include <chrono>
#include <thread>

#include "ipc.h"
#include "ipc_ros_bridge/structure/geometry_msgs_pose.h"

#define TASKNAME "ROS_IPC_Demo"
#define MSGNAME "pose"

int main()
{
    // connect to IPC Central
    // will fail if doesn't find it
    IPC_connect(TASKNAME);

    // define message
    IPC_defineMsg(MSGNAME, IPC_VARIABLE_LENGTH, GEOMETRY_MSGS_POSE_FORMAT);

    geometry_msgs_pose pose;
    pose.orientation.w = -0.861;
    pose.orientation.x = -0.001;
    pose.orientation.y = -0.482;
    pose.orientation.z = 0.161;
    pose.position.x = 0.3;
    pose.position.y = 0.2;
    pose.position.z = 0.1;

    while(true){
        // send the message
        IPC_publishData(MSGNAME, &pose);
        // sleep
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}