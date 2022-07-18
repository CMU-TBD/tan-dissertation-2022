#include "ipc_ros_bridge/ipc_bridge.h"
#include "ros/ros.h"


IPCBridge::IPCBridge(ros::NodeHandle _nh, std::string _taskName)
{
  // IPC's default is really conservative and exits on errors
  IPC_setVerbosity(IPC_Print_Errors);
  Connect(_nh, _taskName);
}

// Deconstructor
IPCBridge::~IPCBridge()
{
  // Disconnect
  Disconnect();
}

void IPCBridge::Connect(std::string _taskName)
{
  Connect(nh, _taskName);
}

void IPCBridge::Connect(ros::NodeHandle _nh, std::string _taskName)
{
  // Start IPC
  auto return_val = IPC_connect(&_taskName[0]);
  nh = _nh;
  taskName = _taskName;
  connected = (return_val == IPC_OK);
}

bool IPCBridge::isConnected()
{
  return connected;
}

void IPCBridge::Disconnect()
{
  if (connected)
  {
    IPC_disconnect();
    connected = false;
  }
}

void IPCBridge::spin()
{
  ros::Duration d(0);
  spin(d);
}

void IPCBridge::spin(ros::Duration duration)
{
  if (!connected){
    ROS_WARN("Attemping to spin while IPC is not connected");
    return;
  }

  auto startTime = ros::Time::now();
  // loop while ros is running
  while (ros::ok() && (duration.isZero() || (ros::Time::now() - startTime) <= duration))
  {
    // dispatch some IPC message
    IPC_listenWait(10);  // 10 milliseconds
    // spin once
    ros::spinOnce();
  }
}