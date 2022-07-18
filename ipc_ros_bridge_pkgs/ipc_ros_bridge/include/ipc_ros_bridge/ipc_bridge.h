
#include <iostream>
#include <unordered_map>
#include <utility>

#include "intermediate_type.h"
#include "ipc.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

template <typename M>
struct SubOps
{
  ros::Subscriber sub;
  IntermediateType<M> *t;
};

template <typename M>
struct ReceiveOps
{
  std::function<void(M)> callback;
  IntermediateType<M> *t;
};

class IPCBridge
{
private:
  bool connected = false;
  ros::NodeHandle nh;
  std::string taskName;
  // TODO: This might cause a memory leak
  std::unordered_map<std::string, void *> opsMap;
  std::unordered_map<std::string, void *> subMap;
  std::unordered_map<std::string, ros::Publisher> pubMap;

public:
  //Constructors 
  IPCBridge(ros::NodeHandle _nh, std::string _taskName);
  
  // Deconstructor
  ~IPCBridge();

  // Non template Methods
  void Connect(ros::NodeHandle _nh, std::string _taskName);
  void Connect(std::string _taskName);
  bool isConnected();
  void Disconnect();
  void spin();
  void spin(ros::Duration duration);

  template <class M, class T>
  void RelayTopicToIPC(std::string ipcMsgName, std::string rosTopicName)
  {
    if (!connected){
      ROS_WARN("Attempt to RelayTopicToIPC when IPC is not connected");
      return;
    }

    // create the intermediate message.
    IntermediateType<M> *t = new T(ipcMsgName);

    // define the message
    IPC_defineMsg(t->getName(), IPC_VARIABLE_LENGTH, t->getFormatString());

    // subscribe to the topic
    boost::function<void(const M &)> callback = [=](const M &msg) {
        t->publishData((void *)&msg);
    };
    // create the ros subscriber
    auto sub = nh.subscribe<M>(rosTopicName, 1, callback);

    // create the sub ops
    SubOps<M> *ops = new SubOps<M>();
    ops->t = t;
    ops->sub = sub;

    // subscribe to it
    subMap.insert(std::make_pair(rosTopicName, (void *)ops));
  }

  template <class M>
  void StopRelayTopicToIPC(std::string topic)
  {
    // first find it
    auto result = subMap.find(topic);
    if (result != subMap.end())
    {
      // get the sub ops
      SubOps<M> *ops = static_cast<SubOps<M> *>(result->second);
      // stop the subscriber in case of race condition
      ops->sub.shutdown();
      // delete/free the instance
      delete ops->t;
      subMap.erase(topic);
      delete ops;
    }
    else
    {
      // the topic doesn't exist
    }
  }

  template <class M, class T>
  void ReceiveTopicFromIPC(std::string ipcMsgName, std::function<void(M)> callback, int queueSize = 1)
  {
    ReceiveTopicFromIPC<M, T>(ipcMsgName, callback, ipcMsgName, queueSize);
  }

  template <class M, class T>
  void ReceiveTopicFromIPC(std::string ipcMsgName, std::function<void(M)> callback, std::string callbackName, int queueSize = 1)
  {
    if (!connected){
      ROS_WARN("Attempt to ReceiveTopicFromIPC when IPC is not connected");
      return;
    }
    
    // create intermediate type
    IntermediateType<M> *t = new T(ipcMsgName);
    // define and register message
    IPC_defineMsg(t->getName(), IPC_VARIABLE_LENGTH, t->getFormatString());
    // Set the message queue size
    IPC_setMsgQueueLength(t->getName(), queueSize);

    // create the publishing ops
    ReceiveOps<M> *ops = new ReceiveOps<M>();
    ops->t = t;
    ops->callback = callback;

    // pass to the static method
    ReceiveTopicFromIPCStatic<M>(ops);

    // add it to map when unsubscribe from it.
    opsMap.insert(std::make_pair(callbackName, (void *)ops));
  }


  template <class M, class T>
  void RelayTopicFromIPC(std::string ipcMsgName, std::string rosTopicName, int queueSize = 1)
  {
    if (!connected){
      ROS_WARN("Attempt to RelayTopicFromIPC when IPC is not connected");
      return;
    }
    // create publisher
    ros::Publisher pub = nh.advertise<M>(rosTopicName, queueSize);
    pubMap.insert(std::make_pair(rosTopicName, pub));

    // create a lambda function that publish it
    auto pubCallback = [=](M msg) {
      auto result = pubMap.find(rosTopicName);
      auto pub = result->second;
      pub.publish(msg);
    };

    ReceiveTopicFromIPC<M, T>(ipcMsgName, pubCallback, rosTopicName, queueSize);
  }

  template <class M>
  bool StopReceiveTopicFromIPC(std::string callbackName)
  {
    // first find it
    auto result = opsMap.find(callbackName);
    if (result != opsMap.end())
    {
      ReceiveOps<M> *ops = static_cast<ReceiveOps<M> *>(result->second);
      // unsubscribe from IPC
      IPC_unsubscribe(ops->t->getName(), StaticCallback<M>);
      // delete/free the instance
      delete ops->t;
      opsMap.erase(callbackName);
      delete ops;
      return true;
    }
    else
    {
      return false;
    }
  }

  template <class M>
  bool StopRelayTopicFromIPC(std::string publishTopic)
  {
    // first find it
    auto result = opsMap.find(publishTopic);
    if (result != opsMap.end())
    {
      opsMap.erase(publishTopic);
      return StopReceiveTopicFromIPC<M>(publishTopic);
    }
    else
    {
      return false;
    }
    
  }

  template <class M>
  static void StaticCallback(MSG_INSTANCE msgRef, void* callData, void *clientData)
  {
    ReceiveOps<M> *ops = (ReceiveOps<M> *)clientData;
    ops->callback(ops->t->ContainerToMessage(callData));
    // Free the data
    IPC_freeData(IPC_msgInstanceFormatter(msgRef), callData);
  }

  template <class M>
  static void ReceiveTopicFromIPCStatic(ReceiveOps<M> *ops)
  {
    // subscribe to it
    IPC_subscribeData(ops->t->getName(), StaticCallback<M>, ops);
  }
};