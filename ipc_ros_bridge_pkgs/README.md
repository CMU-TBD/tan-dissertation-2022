# IPC_ROS_BRIDGE
Copyright - Transportation, Bots, and Disability Lab, Carnegie Mellon University  
Released under MIT License  

This ROS package relays ROS Topics between ROS & [IPC](https://cs.cmu.edu/~ipc) Worlds. 

## How to Install:
1. Compile and build IPC library from [cs.cmu.edu/~ipc](https://cs.cmu.edu/~ipc).
2. Clone this repo into ROS workspace.
3. Set the `IPC_DIR` and `IPC_LIB_PATH` environment variable. The first should be the root of the IPC directory and the second is the path to the `libipc.a` static library.

## How to Run A Relay on ROS that transfer messages to and from IPC:
1. Check if the Message type already exist, if not create the definition of the ROS Topic type you going to use:
    1. First, create a header that defines the IPC string format and `struct` (If any). Then, put it in the `include/structure` folder. You can leave the struct part empty if its a single value (IPC format only has the type, e.g. `double`). 
        ```
            #pragma once

            // Define the struct here
            typedef struct 
            {
                char* Data;
                int Date;
            } std_msgs_string;

            // Define the constants
            const char* std_msgs_string_format  = "{string, int}";
        ```
    2. Create a derived class from `IntermediateType<M>` that converts the data type or `struct` into the correct Message type and vice versa. This is done through two virtual methods, `publishData` and `ContainerToMessage`. Here's an example for `std_msgs/String` and `geometry_msgs/Point` message type.
        ```
        #pragma once
        #include "../intermediate_type.h"
        #include "std_msgs/String.h"
        #include "../structure/std_msgs_string.h"
        #include <iostream>


        class StdMsgsString: public IntermediateType<std_msgs::String>
        {
        public:

            StdMsgsString(const std::string name = STD_MSGS_STRING_NAME)
                :IntermediateType(name, STD_MSGS_STRING_FORMAT){
            }

            virtual void publishData(void* _msg){
                std_msgs::String *msg = (std_msgs::String *) _msg;
                const char* cstr = msg->data.c_str();
                sendToIPC(&cstr);
            }

            virtual std_msgs::String ContainerToMessage(void* _container)
            {
                std_msgs::String msg;
                // create new message
                char ** container = (char **) _container;
                msg.data = std::string(*container);
                return msg;
            }
        };
        ```
        ```
        #pragma once
        #include "../intermediate_type.h"
        #include "geometry_msgs/Point.h"
        #include "../structure/geometry_msgs_point.h"


        class GoemetryMsgsPoint: public IntermediateType<geometry_msgs::Point>
        {
        public:
            
            GoemetryMsgsPoint(const std::string name = GEOMETRY_MSGS_POINT_NAME)
                :IntermediateType(name, GEOMETRY_MSGS_POINT_FORMAT){
            }

            virtual void publishData(void* _msg){

                geometry_msgs::Point *msg = (geometry_msgs::Point *) _msg;
                geometry_msgs_point pointStruct;
                pointStruct.x = msg->x;
                pointStruct.y = msg->y;
                pointStruct.z = msg->z;
                sendToIPC(&pointStruct);
            }

            virtual geometry_msgs::Point ContainerToMessage(void* _container)
            {
                geometry_msgs::Point msg;
                // create new message
                geometry_msgs_point *container = (geometry_msgs_point *) _container;
                msg.x = container->x;
                msg.y = container->y;
                msg.z = container->z;
                return msg;
            }
        };

        ```
2. Write a ROS Node that create the linkage between ROS and IPC.
    ```
        auto ipc = IPCBridge(nh, "ROS_IPC_Demo");
        std::cout << "Connected, Starting Relay ...." << std::endl;
        // Relay Message from IPC to ROS and publish on topic "fromIPC"
        ipc.RelayTopicFromIPC<std_msgs::String, StdMsgsString>("ROSMSG1","fromIPC");
        // Relay the Message from topic "t" to IPC
        ipc.RelayTopicToIPC<std_msgs::String, StdMsgsString>("IPCMSG1","t"); 
        ipc.RelayTopicToIPC<std_msgs::Bool, StdMsgsBool>("IPCMSG2","b"); 

        std::cout << "Starting Wait ...." << std::endl;
        ros::Duration wait(10);
        ipc.spin(wait);

        std::cout << "Stopping Relay ...." << std::endl;

        // Stop Mesage
        ipc.StopRelayTopicFromIPC<std_msgs::String>("fromIPC");
        ipc.StopRelayTopicToIPC<std_msgs::String>("t"); 

        ipc.Disconnect();
    ```
3. For both `RelayTopicToIPC` and `RelayTopicFromIPC`, the first argument is the IPC Message Name and the second argument is the ROS Topic Name.
4. The messages will only be passed once `ipc.spin()` is called. You can also specify a duration with `ipc.spin(ros::Duration d)`.

## How to receive IPC message in C++ Ros Nodes:
Currently, we also support receiving IPC message in C++ ROS Node where the message is transformed into a ROS Message.
```

void receiveMsg(std_msgs::String msg){
    std::cout << "Received:" << msg.data << std::endl;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "receive_node");
    ros::NodeHandle nh;
    auto ipc = IPCBridge(nh, "ros_ipc_receive_example");

    // Specify the ROS Message type to convert to and the converter
    ipc.ReceiveTopicFromIPC<std_msgs::String, StdMsgsString>("ROSMSG1", receiveMsg);
    ipc.spin();
    ipc.Disconnect();
```

## How to Run in IPC.
1. Make sure the compiler has reference to the directory of `include/structure` (example: `-I$(Home)/ros_ws/src/ipc_ros_bridge/include/structure`).
2. When defining the message, use the provided `struct` and `format` in the defined message.
    ```
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
    ```