
#pragma once
#include <iostream>
#include "ipc.h"
#include "ros/ros.h"

template <class M>
class IntermediateType
{
protected:
    std::string name;
    char* format;

    void sendToIPC(void * data){
        IPC_publishData(getName(), data);
    }

public:

    IntermediateType(const std::string _name, const char* _format):
        name(_name)
    {
        format = const_cast<char*>(_format);
    }

    // For ROS -> IPC
    virtual void publishData(void* msg) = 0;
    // For IPC -> ROS
    virtual M ContainerToMessage(void* container) = 0;
    
    char* getName(){
        return &name[0];
    }

    char* getFormatString(){
        return format;
    }
};
