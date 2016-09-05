#ifndef DEVICE_H
#define DEVICE_H

#include <map>
#include <string>
#include <vector>
#include "feature.hpp"
#include "ros/time.h"

namespace Tapi
{
class Device
{
public:
  // Constructor/Destructor
  Device(uint8_t type, std::string name, std::string uuid, unsigned long lastSeq, ros::Time lastSeen,
         unsigned long heartbeat, std::map<std::string, Feature> features);
  ~Device();

  // Public member functions
  bool Active();
  void Deactivate();
  Feature* GetFeatureByUUID(std::string uuid);
  unsigned long GetHeartbeat();
  ros::Time GetLastSeen();
  unsigned long GetLastSeq();
  std::string GetName() const;
  std::vector<Feature*> GetSortedFeatures();
  uint8_t GetType();
  std::string GetUUID();
  void Update(uint8_t type, std::string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
              std::map<std::string, Feature> features);

private:
  // Private member variables
  bool active;
  std::map<std::string, Feature> features;
  unsigned long heartbeat;
  ros::Time lastSeen;
  unsigned long lastSeq;
  std::string name;
  uint8_t type;
  std::string uuid;

  // Private member functions
  static bool compareFeatureNames(const Feature* first, const Feature* second);
};
}

#endif  // DEVICE_H
