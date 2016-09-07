#include "tapicore.hpp"
#include "std_msgs/Time.h"
#include "tapi_lib/Connection.h"
#include "tapi_lib/Device.h"
#include "tapi_lib/Feature.h"
#include "tapi_lib/feature.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

TapiCore::TapiCore(ros::NodeHandle* nh) : nh(nh)
{
  helloServ = nh->advertiseService("/Tapi/HelloServ", &TapiCore::hello, this);
  configPub = nh->advertise<tapi_lib::Connection>("/Tapi/Config", 1000);
  lastChangedPub = nh->advertise<std_msgs::Time>("/Tapi/LastChanged", 5);
  ROS_INFO("Started Hello-Service, ready for connections.");
  heartbeatCheckTimer =
      nh->createTimer(ros::Duration(HEARTBEAT_CHECK_INTERVAL / 1000.0), &TapiCore::heartbeatCheck, this);
  heartbeatCheckTimer.start();
  delSub = nh->subscribe("/Tapi/DeleteConnection", 1000, &TapiCore::deleteConnection, this);
  clearAllSub = nh->subscribe("/Tapi/ClearAll", 1, &TapiCore::clearAll, this);
  clearInactiveSub = nh->subscribe("/Tapi/ClearInactive", 1, &TapiCore::clearInactive, this);
  connectSub = nh->subscribe("/Tapi/ConnectFeatures", 1, &TapiCore::connectFeatures, this);
  getDevsServ = nh->advertiseService("/Tapi/GetDeviceList", &TapiCore::getDevicesSorted, this);
  getConnsServ = nh->advertiseService("/Tapi/GetConnectionList", &TapiCore::getConnectionList, this);
}

TapiCore::~TapiCore()
{
  connectSub.shutdown();
  helloServ.shutdown();
  lastChangedPub.shutdown();
  configPub.shutdown();
  heartbeatCheckTimer.stop();
  delSub.shutdown();
  clearAllSub.shutdown();
  clearInactiveSub.shutdown();
  getDevsServ.shutdown();
  getConnsServ.shutdown();
  ROS_INFO("Hello-Service has been stopped.");
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

void TapiCore::clearAll(const std_msgs::Bool::ConstPtr& cl)
{
  if (cl->data)
  {
    for (auto it = devices.begin(); it != devices.end(); ++it)
    {
      if (it->second.GetType() == tapi_lib::Device::Type_Subscriber)
      {
        vector<Tapi::Feature*> features = it->second.GetSortedFeatures();
        for (auto it2 = features.begin(); it2 != features.end(); ++it2)
        {
          tapi_lib::Connection del;
          del.Coefficient = 1.0;
          del.PublisherFeatureUUID = "0";
          del.PublisherUUID = "0";
          del.SubscriberFeatureUUID = (*it2)->GetUUID();
          del.SubscriberUUID = it->second.GetUUID();
          configPub.publish(del);
        }
      }
    }
    connections.clear();
    devices.clear();
    changed();
  }
}

void TapiCore::clearInactive(const std_msgs::Bool::ConstPtr& cl)
{
  if (cl->data)
  {
    vector<string> toDelete;
    for (auto it = devices.begin(); it != devices.end(); ++it)
    {
      if (!it->second.Active())
      {
        vector<Tapi::Feature*> features = it->second.GetSortedFeatures();
        for (auto it2 = features.begin(); it2 != features.end(); ++it2)
        {
          tapi_lib::Connection del;
          del.Coefficient = 1.0;
          del.PublisherFeatureUUID = "0";
          del.PublisherUUID = "0";
          del.SubscriberFeatureUUID = (*it2)->GetUUID();
          del.SubscriberUUID = it->second.GetUUID();
          configPub.publish(del);
          if (connections.count((*it2)->GetUUID()) > 0)
            connections.erase((*it2)->GetUUID());
        }
        toDelete.push_back(it->first);
      }
    }
    for (auto it = toDelete.begin(); it != toDelete.end(); ++it)
      devices.erase(*it);
  }
  changed();
}

bool TapiCore::compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second)
{
  return first->GetName() < second->GetName();
}

void TapiCore::connectFeatures(const tapi_lib::Connect::ConstPtr& con)
{
  string feature1uuid, feature2uuid;
  double coefficient;
  feature1uuid = con->Feature1UUID;
  feature2uuid = con->Feature2UUID;
  coefficient = con->Coefficient;
  Tapi::Device *device1, *device2;
  device1 = getDeviceByFeatureUUID(feature1uuid);
  device2 = getDeviceByFeatureUUID(feature2uuid);
  if (device1 == 0 || device2 == 0)
    // At least one Device not found
    return;
  if (device1->GetType() == device2->GetType())
    // Cannont connect devices of same type (publisher-publisher or subscriber-subscriber)
    return;
  if (device1->GetFeatureByUUID(feature1uuid)->GetType() != device2->GetFeatureByUUID(feature2uuid)->GetType())
    // Cannot connect features of different message-types
    return;

  // Who is publisher, who subscriber?
  string publisherUUID, subscriberUUID, publisherFeatureUUID, subscriberFeatureUUID;
  if (device1->GetType() == tapi_lib::Device::Type_Subscriber)
  {
    subscriberUUID = device1->GetUUID();
    subscriberFeatureUUID = feature1uuid;
    publisherUUID = device2->GetUUID();
    publisherFeatureUUID = feature2uuid;
  }
  else
  {
    subscriberUUID = device2->GetUUID();
    subscriberFeatureUUID = feature2uuid;
    publisherUUID = device1->GetUUID();
    publisherFeatureUUID = feature1uuid;
  }

  if (connections.count(subscriberFeatureUUID) == 0)
  {
    // Old connection was removed before reassigning so connect devices/features
    Tapi::Connection connection(publisherUUID, publisherFeatureUUID, subscriberUUID, subscriberFeatureUUID,
                                coefficient);
    connections.emplace(subscriberFeatureUUID, connection);
    changed();
  }
}

bool TapiCore::getConnectionList(tapi_lib::GetConnectionList::Request& listReq,
                                 tapi_lib::GetConnectionList::Response& listResp)
{
  if (listReq.Get)
  {
    vector<Tapi::Connection*> connectionVec;
    for (auto it = connections.begin(); it != connections.end(); ++it)
      connectionVec.push_back(&it->second);
    vector<tapi_lib::Connection> msg;
    for (auto it = connectionVec.begin(); it != connectionVec.end(); ++it)
    {
      tapi_lib::Connection connection;
      connection.Coefficient = (*it)->GetCoefficient();
      connection.PublisherFeatureUUID = (*it)->GetPublisherFeatureUUID();
      connection.PublisherUUID = (*it)->GetPublisherUUID();
      connection.SubscriberFeatureUUID = (*it)->GetSubscriberFeatureUUID();
      connection.SubscriberUUID = (*it)->GetSubscriberUUID();
      msg.push_back(connection);
    }
    listResp.Connections = msg;
    return true;
  }
  else
    return false;
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

void TapiCore::deleteConnection(const std_msgs::String::ConstPtr& del)
{
  string subscriberFeatureUUID = del->data;
  deleteConnection(subscriberFeatureUUID);
}

void TapiCore::deleteConnection(std::string subscriberFeatureUUID)
{
  if (connections.count(subscriberFeatureUUID) > 0)
  {
    Tapi::Connection* connection = &connections.at(subscriberFeatureUUID);
    string subscriberUUID = connection->GetSubscriberUUID();
    tapi_lib::Connection msg;
    msg.PublisherUUID = "0";
    msg.PublisherFeatureUUID = "0";
    msg.SubscriberUUID = subscriberUUID;
    msg.SubscriberFeatureUUID = subscriberFeatureUUID;
    msg.Coefficient = 0;
    configPub.publish(msg);
    connections.erase(subscriberFeatureUUID);
  }
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

bool TapiCore::getDevicesSorted(tapi_lib::GetDeviceList::Request& listReq, tapi_lib::GetDeviceList::Response& listResp)
{
  if (listReq.Get)
  {
    vector<Tapi::Device*> devicesList;
    for (auto it = devices.begin(); it != devices.end(); ++it)
      devicesList.push_back(&it->second);
    if (devicesList.size() > 1)
      sort(devicesList.begin(), devicesList.end(), compareDeviceNames);
    vector<tapi_lib::Device> answer;
    for (auto it = devicesList.begin(); it != devicesList.end(); ++it)
    {
      tapi_lib::Device device;
      device.Active = (*it)->Active();
      device.DeviceType = (*it)->GetType();
      device.Heartbeat = (*it)->GetHeartbeat();
      device.LastSeen = (*it)->GetLastSeen();
      device.LastSeq = (*it)->GetLastSeq();
      device.Name = (*it)->GetName();
      device.UUID = (*it)->GetUUID();
      vector<Feature*> features = (*it)->GetSortedFeatures();
      vector<tapi_lib::Feature> featureMsgs;
      for (auto it2 = features.begin(); it2 != features.end(); ++it2)
      {
        tapi_lib::Feature featureMsg;
        featureMsg.FeatureType = (*it2)->GetType();
        featureMsg.Name = (*it2)->GetName();
        featureMsg.UUID = (*it2)->GetUUID();
        featureMsgs.push_back(featureMsg);
      }
      device.Features = featureMsgs;
      answer.push_back(device);
    }
    listResp.Devices = answer;
    return true;
  }
  else
    return false;
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

bool TapiCore::hello(tapi_lib::Hello::Request& helloReq, tapi_lib::Hello::Response& helloResp)
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
    helloResp.Status = tapi_lib::HelloResponse::StatusOK;
    helloResp.Heartbeat = heartbeat;
    changed();
    return true;
  }
  else if (devices.count(uuid) == 1)
  {
    devices.at(uuid).Update(type, name, lastSeq, lastSeen, heartbeat, features);
    helloResp.Status = tapi_lib::HelloResponse::StatusOK;
    helloResp.Heartbeat = heartbeat;
    changed();
    return true;
  }
  else
  {
    ROS_ERROR("Hello message couldn't be decoded, looks like there is something wrong with the devices database. "
              "Please try to restart the Hello-Service.");
    helloResp.Status = tapi_lib::HelloResponse::StatusError;
    helloResp.Heartbeat = STANDARD_HEARTBEAT_INTERVAL;
    return false;
  }
  return false;
}

void TapiCore::sendAllConnections()
{
  for (auto it = connections.begin(); it != connections.end(); ++it)
  {
    tapi_lib::Connection msg;
    msg.PublisherUUID = it->second.GetPublisherUUID();
    msg.PublisherFeatureUUID = it->second.GetPublisherFeatureUUID();
    msg.SubscriberUUID = it->second.GetSubscriberUUID();
    msg.SubscriberFeatureUUID = it->second.GetSubscriberFeatureUUID();
    msg.Coefficient = it->second.GetCoefficient();
    configPub.publish(msg);
  }
}
}
