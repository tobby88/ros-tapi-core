#ifndef TAPICORE_H
#define TAPICORE_H

// Intervals in ms
#define HEARTBEAT_CHECK_INTERVAL 100L
#define STANDARD_HEARTBEAT_INTERVAL 2000L

#include <map>
#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "ros/timer.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tapi_lib/Connect.h"
#include "tapi_lib/GetConnectionList.h"
#include "tapi_lib/GetDeviceList.h"
#include "tapi_lib/Hello.h"
#include "tapi_lib/connection.hpp"
#include "tapi_lib/device.hpp"

namespace Tapi
{
class TapiCore
{
public:
  // Constructor/Destructor
  explicit TapiCore(ros::NodeHandle* nh);
  ~TapiCore();

private:
  // Private member variables
  ros::Subscriber clearSub;
  ros::Publisher configPub;
  std::map<std::string, Tapi::Connection> connections;
  ros::Subscriber connectSub;
  ros::Subscriber delSub;
  std::map<std::string, Tapi::Device> devices;
  ros::ServiceServer getConnsServ;
  ros::ServiceServer getDevsServ;
  ros::Timer heartbeatCheckTimer;
  ros::ServiceServer helloServ;
  ros::Publisher lastChangedPub;
  ros::NodeHandle* nh;

  // Private member functions
  void changed();
  void clear(const std_msgs::Bool::ConstPtr& cl);
  static bool compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second);
  void connectFeatures(const tapi_lib::Connect::ConstPtr& con);
  void debugOutput();
  void deleteConnection(const std_msgs::String::ConstPtr& del);
  void deleteConnection(std::string subscriberFeatureUUID);
  bool getConnectionList(tapi_lib::GetConnectionList::Request& listReq,
                         tapi_lib::GetConnectionList::Response& listResp);
  Tapi::Device* getDeviceByFeatureUUID(std::string uuid);
  bool getDevicesSorted(tapi_lib::GetDeviceList::Request& listReq, tapi_lib::GetDeviceList::Response& listResp);
  void heartbeatCheck(const ros::TimerEvent& e);
  bool hello(tapi_lib::Hello::Request& helloReq, tapi_lib::Hello::Response& helloResp);
  void sendAllConnections();
};
}

#endif  // TAPICORE_H
