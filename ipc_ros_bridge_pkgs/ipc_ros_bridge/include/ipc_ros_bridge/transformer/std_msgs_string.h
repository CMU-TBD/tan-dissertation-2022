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
