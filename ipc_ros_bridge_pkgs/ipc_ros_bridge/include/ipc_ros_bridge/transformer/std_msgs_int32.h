#pragma once
#include "../intermediate_type.h"
#include "std_msgs/Int32.h"
#include "../structure/std_msgs_int32.h"


class StdMsgsInt32: public IntermediateType<std_msgs::Int32>
{
public:
    
    StdMsgsInt32(const std::string name = STD_MSGS_INT32_NAME)
        :IntermediateType(name, STD_MSGS_INT32_FORMAT){
    }

    virtual void publishData(void* _msg){
        std_msgs::Int32 *msg = (std_msgs::Int32 *) _msg;
        sendToIPC(&(msg->data));
    }

    virtual std_msgs::Int32 ContainerToMessage(void* _container)
    {
        std_msgs::Int32 msg;
        // create new message
        int* Data = (int*) _container;
        msg.data = *Data;
        return msg;
    }
};
