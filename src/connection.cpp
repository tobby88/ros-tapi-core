#include "connection.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Connection::Connection(string publisherUUID, string publisherFeatureUUID, string subscriberUUID,
                       string subscriberFeatureUUID, double coefficient)
  : publisherUUID(publisherUUID)
  , publisherFeatureUUID(publisherFeatureUUID)
  , subscriberUUID(subscriberUUID)
  , subscriberFeatureUUID(subscriberFeatureUUID)
  , coefficient(coefficient)
{
}

Connection::~Connection()
{
}

// Public member functions

double Connection::GetCoefficient()
{
  return coefficient;
}

string Connection::GetPublisherFeatureUUID()
{
  return publisherFeatureUUID;
}

string Connection::GetPublisherUUID()
{
  return publisherUUID;
}

string Connection::GetSubscriberFeatureUUID()
{
  return subscriberFeatureUUID;
}

string Connection::GetSubscriberUUID()
{
  return subscriberUUID;
}
}
