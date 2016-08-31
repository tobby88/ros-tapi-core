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
  void Clear();
  bool ConnectFeatures(std::string feature1UUID, std::string feature2UUID, double coefficient);
  void DebugOutput();
  bool DeleteConnection(std::string receiverFeatureUUID);
  std::vector<Tapi::Connection*> GetConnections();
  std::vector<Tapi::Device*> GetDevicesSorted();

private:
  // Private member variables
  ros::Publisher configPub;
  std::map<std::string, Tapi::Connection> connections;
  std::map<std::string, Tapi::Device> devices;
  ros::Timer heartbeatCheckTimer;
  ros::ServiceServer helloServ;
  ros::Publisher lastChangedPub;
  ros::NodeHandle* nh;

  // Private member functions
  void changed();
  static bool compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second);
  Tapi::Device* getDeviceByFeatureUUID(std::string uuid);
  void heartbeatCheck(const ros::TimerEvent& e);
  bool hello(tapi_msgs::Hello::Request& helloReq, tapi_msgs::Hello::Response& helloResp);
  void sendAllConnections();
};
}

#endif  // TAPICORE_H
