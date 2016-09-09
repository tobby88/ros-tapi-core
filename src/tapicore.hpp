/******************************************************************************
*  This file is part of tapi_core.                                            *
*                                                                             *
*  tapi_core is free software: you can redistribute it and/or modify          *
*  it under the terms of the GNU General Public License as published by       *
*  the Free Software Foundation, either version 3 of the License, or          *
*  (at your option) any later version.                                        *
*                                                                             *
*  tapi_core is distributed in the hope that it will be useful,               *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*  GNU General Public License for more details.                               *
*                                                                             *
*  You should have received a copy of the GNU General Public License          *
*  along with tapi_core.  If not, see <http://www.gnu.org/licenses/>.         *
*                                                                             *
*  Diese Datei ist Teil von tapi_core.                                        *
*                                                                             *
*  tapi_core ist Freie Software: Sie können es unter den Bedingungen          *
*  der GNU General Public License, wie von der Free Software Foundation,      *
*  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                 *
*  veröffentlichten Version, weiterverbreiten und/oder modifizieren.          *
*                                                                             *
*  tapi_core wird in der Hoffnung, dass es nützlich sein wird, aber           *
*  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite         *
*  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK  *
*  Siehe die GNU General Public License für weitere Details.                  *
*                                                                             *
*  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem  *
*  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. *
*******************************************************************************/

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
  ros::Subscriber clearAllSub;
  ros::Subscriber clearInactiveSub;
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
  void clearAll(const std_msgs::Bool::ConstPtr& cl);
  void clearInactive(const std_msgs::Bool::ConstPtr& cl);
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
