#pragma once
#include "../intermediate_type.h"
#include "std_msgs/Bool.h"
#include "../structure/std_msgs_bool.h"


class StdMsgsBool: public IntermediateType<std_msgs::Bool>
{
public:
    
    StdMsgsBool(const std::string name = STD_MSGS_BOOL_NAME)
        :IntermediateType(name, STD_MSGS_BOOL_FORMAT){
    }


    virtual void publishData(void* _msg){
        std_msgs::Bool *msg = (std_msgs::Bool *) _msg;
        int intBool = msg->data ? 1 : 0;
        sendToIPC(&(intBool));
    }

    virtual std_msgs::Bool ContainerToMessage(void* _container)
    {
        std_msgs::Bool msg;
        // create new message
        int* Data = (int*) _container;
        msg.data = (*Data == 1 ? true : false); 
        return msg;
    }
};
