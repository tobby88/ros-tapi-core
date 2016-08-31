#include "tapicore.hpp"
#include "feature.hpp"
#include "std_msgs/Time.h"
#include "tapi_msgs/Config.h"
#include "tapi_msgs/Feature.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

TapiCore::TapiCore(ros::NodeHandle* nh) : nh(nh)
{
  helloServ = nh->advertiseService("Tapi/HelloServ", &TapiCore::hello, this);
  configPub = nh->advertise<tapi_msgs::Config>("Tapi/Config", 1000);
  lastChangedPub = nh->advertise<std_msgs::Time>("Tapi/LastChanged", 5);
  ROS_INFO("Started Hello-Service, ready for connections.");
  heartbeatCheckTimer =
      nh->createTimer(ros::Duration(HEARTBEAT_CHECK_INTERVAL / 1000.0), &TapiCore::heartbeatCheck, this);
  heartbeatCheckTimer.start();
  clearSub = nh->subscribe("Tapi/Clear", 1, &TapiCore::clear, this);
}

TapiCore::~TapiCore()
{
  helloServ.shutdown();
  lastChangedPub.shutdown();
  configPub.shutdown();
  heartbeatCheckTimer.stop();
  clearSub.shutdown();
  ROS_INFO("Hello-Service has been stopped.");
}

// Public member functions

bool TapiCore::ConnectFeatures(string feature1uuid, string feature2uuid, double coefficient)
{
  Tapi::Device *device1, *device2;
  device1 = getDeviceByFeatureUUID(feature1uuid);
  device2 = getDeviceByFeatureUUID(feature2uuid);
  if (device1 == 0 || device2 == 0)
    // At least one Device not found
    return false;
  if (device1->GetType() == device2->GetType())
    // Cannont connect devices of same type (sender-sender or receiver-receiver)
    return false;
  if (device1->GetFeatureByUUID(feature1uuid)->GetType() != device2->GetFeatureByUUID(feature2uuid)->GetType())
    // Cannot connect features of different types
    return false;

  // Who is sender, who receiver?
  string senderUUID, receiverUUID, senderFeatureUUID, receiverFeatureUUID;
  if (device1->GetType() == tapi_msgs::HelloRequest::Type_ReceiverDevice)
  {
    receiverUUID = device1->GetUUID();
    receiverFeatureUUID = feature1uuid;
    senderUUID = device2->GetUUID();
    senderFeatureUUID = feature2uuid;
  }
  else
  {
    receiverUUID = device2->GetUUID();
    receiverFeatureUUID = feature2uuid;
    senderUUID = device1->GetUUID();
    senderFeatureUUID = feature1uuid;
  }

  if (connections.count(receiverFeatureUUID) > 0)
    // Old connection was not removed before reassigning!
    return false;
  else
  {
    // Connect devices/features
    Tapi::Connection connection(senderUUID, senderFeatureUUID, receiverUUID, receiverFeatureUUID, coefficient);
    connections.emplace(receiverFeatureUUID, connection);
    device1->GetFeatureByUUID(feature1uuid)->IncrementConnections();
    device2->GetFeatureByUUID(feature2uuid)->IncrementConnections();
    changed();
    return true;
  }
  // "Error" handler - should never be reached:
  return false;
}

bool TapiCore::DeleteConnection(string receiverFeatureUUID)
{
  if (connections.count(receiverFeatureUUID) > 0)
  {
    Tapi::Connection* connection = &connections.at(receiverFeatureUUID);
    string senderUUID = connection->GetSenderUUID();
    string senderFeatureUUID = connection->GetSenderFeatureUUID();
    string receiverUUID = connection->GetReceiverUUID();
    if (devices.count(senderUUID) > 0)
      devices.at(senderUUID).GetFeatureByUUID(senderFeatureUUID)->DecrementConnections();
    if (devices.count(receiverUUID) > 0)
      devices.at(receiverUUID).GetFeatureByUUID(receiverFeatureUUID)->DecrementConnections();
    tapi_msgs::Config msg;
    msg.SenderUUID = "0";
    msg.SenderFeatureUUID = "0";
    msg.ReceiverUUID = receiverUUID;
    msg.ReceiverFeatureUUID = receiverFeatureUUID;
    msg.Coefficient = 0;
    configPub.publish(msg);
    connections.erase(receiverFeatureUUID);
  }
}

vector<Tapi::Connection*> TapiCore::GetConnections()
{
  vector<Tapi::Connection*> connectionList;
  for (auto it = connections.begin(); it != connections.end(); ++it)
    connectionList.push_back(&it->second);
  return connectionList;
}

vector<Tapi::Device*> TapiCore::GetDevicesSorted()
{
  vector<Tapi::Device*> devicesList;
  for (auto it = devices.begin(); it != devices.end(); ++it)
    devicesList.push_back(&it->second);
  if (devicesList.size() > 1)
    sort(devicesList.begin(), devicesList.end(), compareDeviceNames);
  return devicesList;
}

// Private memeber functions

void TapiCore::changed()
{
  std_msgs::Time timemsg;
  timemsg.data = ros::Time::now();
  lastChangedPub.publish(timemsg);
  sendAllConnections();
#ifdef DEBUG
  debugOutput();
#endif
}

void TapiCore::clear(const std_msgs::Bool::ConstPtr& cl)
{
  if (cl->data)
  {
    for (auto it = connections.begin(); it != connections.end(); ++it)
      DeleteConnection(it->second.GetReceiverFeatureUUID());
    connections.clear();
    devices.clear();
    changed();
  }
}

bool TapiCore::compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second)
{
  return first->GetName() < second->GetName();
}

void TapiCore::debugOutput()
{
  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    ROS_INFO("Debug: Device-Element UUID: %s", it->first.c_str());
    ROS_INFO("Debug: Device-Data: Type: %d, Name: %s, UUID: %s, Last Seq: %lu, Last Seen: %f, Heartbeat-Interval: %lu",
             (unsigned int)it->second.GetType(), it->second.GetName().c_str(), it->second.GetUUID().c_str(),
             it->second.GetLastSeq(), it->second.GetLastSeen().toSec(), it->second.GetHeartbeat());
    vector<Tapi::Feature*> features = it->second.GetSortedFeatures();
    for (auto it2 = features.begin(); it2 != features.end(); ++it2)
    {
      ROS_INFO("Debug: Device-Feature: ID: %s, Feature-Type: %s, Feature-Name: %s", (*it2)->GetUUID().c_str(),
               (*it2)->GetType().c_str(), (*it2)->GetName().c_str());
    }
  }
  // TODO: Print connections
}

Tapi::Device* TapiCore::getDeviceByFeatureUUID(string uuid)
{
  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    if (it->second.GetFeatureByUUID(uuid))
      return &(it->second);
  }
  return 0;
}

void TapiCore::heartbeatCheck(const ros::TimerEvent& e)
{
  bool deactivatedDevices = false;
  for (auto it = devices.begin(); it != devices.end(); ++it)
    if ((it->second.Active()) &&
        (ros::Time::now().toSec() - it->second.GetLastSeen().toSec() > 2.5 * STANDARD_HEARTBEAT_INTERVAL / 1000.0))
    {
      it->second.Deactivate();
      deactivatedDevices = true;
    }
  if (deactivatedDevices)
    changed();
}

bool TapiCore::hello(tapi_msgs::Hello::Request& helloReq, tapi_msgs::Hello::Response& helloResp)
{
  string uuid = helloReq.UUID;
  unsigned long lastSeq = helloReq.Header.seq;
  ros::Time lastSeen = helloReq.Header.stamp;
  string name = helloReq.Name;
  uint8_t type = helloReq.DeviceType;
  unsigned long heartbeat = STANDARD_HEARTBEAT_INTERVAL;
  map<string, Tapi::Feature> features;
  for (unsigned int i = 0; i < helloReq.Features.capacity(); i++)
  {
    Tapi::Feature feature(helloReq.Features[i].FeatureType, helloReq.Features[i].Name, helloReq.Features[i].UUID);
    if (features.count(feature.GetUUID()) == 0)
      features.emplace(feature.GetUUID(), feature);
  }
  if (devices.empty() || devices.count(uuid) == 0)
  {
    Tapi::Device device(type, name, uuid, lastSeq, lastSeen, heartbeat, features);
    devices.emplace(uuid, device);
    helloResp.Status = tapi_msgs::HelloResponse::StatusOK;
    helloResp.Heartbeat = heartbeat;
    changed();
    return true;
  }
  else if (devices.count(uuid) == 1)
  {
    devices.at(uuid).Update(type, name, lastSeq, lastSeen, heartbeat, features);
    helloResp.Status = tapi_msgs::HelloResponse::StatusOK;
    helloResp.Heartbeat = heartbeat;
    changed();
    return true;
  }
  else
  {
    ROS_ERROR("Hello message couldn't be decoded, looks like there is something wrong with the devices database. "
              "Please try to restart the Hello-Service.");
    helloResp.Status = tapi_msgs::HelloResponse::StatusError;
    helloResp.Heartbeat = STANDARD_HEARTBEAT_INTERVAL;
    return false;
  }
  return false;
}

void TapiCore::sendAllConnections()
{
  for (auto it = connections.begin(); it != connections.end(); ++it)
  {
    tapi_msgs::Config msg;
    msg.SenderUUID = it->second.GetSenderUUID();
    msg.SenderFeatureUUID = it->second.GetSenderFeatureUUID();
    msg.ReceiverUUID = it->second.GetReceiverUUID();
    msg.ReceiverFeatureUUID = it->second.GetReceiverFeatureUUID();
    msg.Coefficient = it->second.GetCoefficient();
    configPub.publish(msg);
  }
}
}
