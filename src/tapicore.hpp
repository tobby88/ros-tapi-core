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
  bool ConnectFeatures(std::string feature1UUID, std::string feature2UUID, double coefficient);
  bool DeleteConnection(std::string receiverFeatureUUID);
  std::vector<Tapi::Connection*> GetConnections();
  std::vector<Tapi::Device*> GetDevicesSorted();

private:
  // Private member variables
  ros::Subscriber clearSub;
  ros::Publisher configPub;
  std::map<std::string, Tapi::Connection> connections;
  std::map<std::string, Tapi::Device> devices;
  ros::Timer heartbeatCheckTimer;
  ros::ServiceServer helloServ;
  ros::Publisher lastChangedPub;
  ros::NodeHandle* nh;

  // Private member functions
  void changed();
  void clear(const std_msgs::Bool::ConstPtr& cl);
  static bool compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second);
  void debugOutput();
  Tapi::Device* getDeviceByFeatureUUID(std::string uuid);
  void heartbeatCheck(const ros::TimerEvent& e);
  bool hello(tapi_msgs::Hello::Request& helloReq, tapi_msgs::Hello::Response& helloResp);
  void sendAllConnections();
};
}

#endif  // TAPICORE_H
