#include <iostream>
#include <chrono>
#include <thread>

#include "ipc.h"
#include "ipc_ros_bridge/structure/std_msgs_string.h"

#define TASKNAME "ROS_IPC_Demo"
#define MSGNAME "ROSMSG1"

int main()
{
    // connect to IPC Central
    // will fail if doesn't find it
    IPC_connect(TASKNAME);

    // define message
    IPC_defineMsg(MSGNAME, IPC_VARIABLE_LENGTH, STD_MSGS_STRING_FORMAT);

    // create data type
    std::string tmp = "Hello from IPC World";
    const char *str = tmp.c_str();
    
    while(true){
        // send the message
        IPC_publishData(MSGNAME, &str);
        // sleep
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}