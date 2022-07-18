#pragma once
#include "../intermediate_type.h"
#include "std_msgs/Float64.h"
#include "../structure/std_msgs_float64.h"


class StdMsgsFloat64: public IntermediateType<std_msgs::Float64>
{
public:
    
    StdMsgsFloat64(const std::string name = STD_MSGS_FLOAT64_NAME)
        :IntermediateType(name, STD_MSGS_FLOAT64_FORMAT){
    }

    virtual void publishData(void* _msg){
        std_msgs::Float64 *msg = (std_msgs::Float64 *) _msg;
        sendToIPC(&(msg->data));
    }

    virtual std_msgs::Float64 ContainerToMessage(void* _container)
    {
        std_msgs::Float64 msg;
        // create new message
        double* Data = (double*) _container;
        msg.data = *Data;
        return msg;
    }
};
