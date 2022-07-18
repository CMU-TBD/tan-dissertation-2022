

#include <iostream>

#include "ipc.h"
#include "ipc_ros_bridge/structure/std_msgs_string.h"
#include "ipc_ros_bridge/structure/std_msgs_float64.h"

#define TASKNAME "ROS_IPC_Demo"
#define MSGNAME "IPCMSG1"


#define MSGNAME2 "IPCMSG2"

static void msgHandler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
                       void *clientData)
{
    // location to save the data
    char** data = (char**) callData;
    // print data
    std::cout << *data << std::endl;

    // free up the used data
    // IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), &data);
    IPC_freeByteArray(callData);
}

static void msgHandler2(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
                       void *clientData)
{
    // location to save the data
    double* data = (double*) callData;
    // print data
    std::cout << *data << std::endl;

    // free up the used data
    // IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), &data);
    IPC_freeByteArray(callData);
}

int main()
{

    // connect to IPC Central
    // will fail if doesn't fine it
    IPC_connect(TASKNAME);

    // define message
    IPC_defineMsg(MSGNAME, IPC_VARIABLE_LENGTH, STD_MSGS_STRING_FORMAT);
    IPC_defineMsg(MSGNAME2, IPC_VARIABLE_LENGTH, STD_MSGS_FLOAT64_FORMAT);


    // subscribe to message
    IPC_subscribeData(MSGNAME, msgHandler, (void *)TASKNAME);
    IPC_subscribeData(MSGNAME2, msgHandler2, (void *)TASKNAME);

    // run forever 
    IPC_dispatch();
}