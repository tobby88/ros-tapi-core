#ifndef CONNECTION_H
#define CONNECTION_H

#include <string>

namespace Tapi
{
class Connection
{
public:
  // Constructor/Destructor
  Connection(std::string publisherUUID, std::string publisherFeatureUUID, std::string subscriberUUID,
             std::string subscriberFeatureUUID, double coefficient);
  ~Connection();

  // Public member functions
  double GetCoefficient();
  std::string GetPublisherFeatureUUID();
  std::string GetPublisherUUID();
  std::string GetSubscriberFeatureUUID();
  std::string GetSubscriberUUID();

private:
  // Private member variables
  double coefficient;
  std::string publisherFeatureUUID;
  std::string publisherUUID;
  std::string subscriberFeatureUUID;
  std::string subscriberUUID;
};
}

#endif  // CONNECTION_H
