#ifndef TAPICORE_H
#define TAPICORE_H

// Intervals in ms
#define HEARTBEAT_CHECK_INTERVAL 100L
#define STANDARD_HEARTBEAT_INTERVAL 2000L

#include <map>
#include <string>
#include <vector>
#include "connection.hpp"
#include "device.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tapi_msgs/Connect.h"
#include "tapi_msgs/GetDeviceList.h"
#include "tapi_msgs/Hello.h"

namespace Tapi
{
class TapiCore
{
public:
  // Constructor/Destructor
  explicit TapiCore(ros::NodeHandle* nh);
  ~TapiCore();

  // Public member functions
  std::vector<Tapi::Connection*> GetConnections();

private:
  // Private member variables
  ros::Subscriber clearSub;
  ros::Publisher configPub;
  std::map<std::string, Tapi::Connection> connections;
  ros::Subscriber connectSub;
  ros::Subscriber delSub;
  std::map<std::string, Tapi::Device> devices;
  ros::ServiceServer getDevsServ;
  ros::Timer heartbeatCheckTimer;
  ros::ServiceServer helloServ;
  ros::Publisher lastChangedPub;
  ros::NodeHandle* nh;

  // Private member functions
  void changed();
  void clear(const std_msgs::Bool::ConstPtr& cl);
  static bool compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second);
  void connectFeatures(const tapi_msgs::Connect::ConstPtr& con);
  void debugOutput();
  void deleteConnection(const std_msgs::String::ConstPtr& del);
  void deleteConnection(std::string receiverFeatureUUID);
  Tapi::Device* getDeviceByFeatureUUID(std::string uuid);
  bool getDevicesSorted(tapi_msgs::GetDeviceList::Request& listReq, tapi_msgs::GetDeviceList::Response& listResp);
  void heartbeatCheck(const ros::TimerEvent& e);
  bool hello(tapi_msgs::Hello::Request& helloReq, tapi_msgs::Hello::Response& helloResp);
  void sendAllConnections();
};
}

#endif  // TAPICORE_H
