/******************************************************************************
 *  Copyright (C) 2016 by Tobias Holst                                        *
 *                                                                            *
 *  This file is part of tapi_core.                                           *
 *                                                                            *
 *  tapi_core is free software: you can redistribute it and/or modify         *
 *  it under the terms of the GNU General Public License as published by      *
 *  the Free Software Foundation, either version 3 of the License, or         *
 *  (at your option) any later version.                                       *
 *                                                                            *
 *  tapi_core is distributed in the hope that it will be useful,              *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *  GNU General Public License for more details.                              *
 *                                                                            *
 *  You should have received a copy of the GNU General Public License         *
 *  along with tapi_core.  If not, see <http://www.gnu.org/licenses/>.        *
 *                                                                            *
 *  Diese Datei ist Teil von tapi_core.                                       *
 *                                                                            *
 *  tapi_core ist Freie Software: Sie können es unter den Bedingungen         *
 *  der GNU General Public License, wie von der Free Software Foundation,     *
 *  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                *
 *  veröffentlichten Version, weiterverbreiten und/oder modifizieren.         *
 *                                                                            *
 *  tapi_core wird in der Hoffnung, dass es nützlich sein wird, aber          *
 *  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite        *
 *  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK *
 *  Siehe die GNU General Public License für weitere Details.                 *
 *                                                                            *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem *
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.*
 ******************************************************************************/

/*!
 * \file tapicore.hpp
 * \ingroup tapi_core
 * \author Tobias Holst
 * \date 18 Nov 2015
 * \brief Declaration of the Tapi::TapiCore-class and definition of its member variables
 */

#ifndef TAPICORE_H
#define TAPICORE_H

// Intervals in ms
#define HEARTBEAT_CHECK_INTERVAL 100L
#define STANDARD_HEARTBEAT_INTERVAL 2000L

//#define DEBUG

#include <map>
#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
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
/*!
 * \brief Core of Tapi
 *
 * This class is the core of Tapi. It provides the Hello-service for devices to connect to the (T)api
 * It subscribes a few config channels to control the core (e.g. with a gui) and pulishes a config channel to tell the
 * connected devices/nodes what to subscribe.
 * Also it provides services to get information about linked devices and its connections.
 *
 * \author Tobias Holst
 * \version 2.4.0
 */
class TapiCore
{
public:
  /*!
   * \brief Create a TapiCore object. There can be only one in a ros domain!
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   */
  explicit TapiCore(ros::NodeHandle* nh);
  //! Shutdown all tasks and subscribers when closing the core and deleting the object
  ~TapiCore();

private:
  // Private member variables

  /*!
   * \brief Subscriber to clear all data stored in the core (devices and its connections). Listens on \c /Tapi/ClearAll
   * \see Tapi::TapiCore::clearAll
   */
  ros::Subscriber clearAllSub;

  /*!
   * \brief Subscriber to clear all inactive devices and its connections. Listens on \c /Tapi/ClearInactive
   * \see Tapi::TapiCore::clearInactive
   */
  ros::Subscriber clearInactiveSub;

  /*!
   * \brief Publisher to send connection information (which nodes shall subscribe to which other's topics). Publishes
   * on \c /Tapi/Config
   * \see Tapi::TapiCore::sendAllConnections
   */
  ros::Publisher configPub;

  /*!
   * \brief Stores all connections between nodes
   *
   * As the key the uuid of the subscriber's topic is used, since it is unique (a subscriber can only subscribe one
   * topic).
   * The value is the Tapi::Connection object, where all data about the connection (device ids and their feature uuids)
   * is stored.
   * \see \c Tapi::Connection in \c tapi_lib
   */
  std::map<std::string, Tapi::Connection> connections;

  /*!
   * \brief Subscriber to get commands (e.g. from a gui) which nodes shall be connected. Listens on \c
   * /Tapi/ConnectFeatures
   * \see Tapi::TapiCore::connectFeatures
   */
  ros::Subscriber connectSub;

  /*!
   * \brief Subscriber to get commands (e.g. from a gui) which node's connection shall be deleted. Listens on \c
   * /Tapi/DeleteConnection
   * \see Tapi::TapiCore::deleteConnection
   */
  ros::Subscriber delSub;

  /*!
   * \brief Stores all devices connected to the core
   *
   * The key is the device's uuid.
   * The value is the Tapi::Device object, where all data about the device (uuid, features, ...) are stored.
   * \see \c Tapi::Device in \c tapi_lib
   */
  std::map<std::string, Tapi::Device> devices;

  /*!
   * \brief Service to get a list of all connections between (Tapi-compatible) devices. Service has to be called on \c
   * /Tapi/GetConnectionList
   * \see Tapi::TapiCore::getConnectionList
   */
  ros::ServiceServer getConnsServ;

  /*!
   * \brief Service to get a alphabetically sorted list of all devices conntected to the core. Service has to be called
   * on \c /Tapi/GetDeviceList
   * \see Tapi::TapiCore::getDevicesSorted
   */
  ros::ServiceServer getDevsServ;

  /*!
   * \brief Every cycle (cycle length is defined in HEARTBEAT_CHECK_INTERVAL) it calls Tapi::TapiCOre::heartbeatCheck to
   * see if devices lost their connection to the core.
   * \see Tapi::TapiCore::heartbeatCheck
   */
  ros::Timer heartbeatCheckTimer;

  /*!
   * \brief Hello-service to connect devices to the core
   * \see Tapi::TapiCore::hello
   */
  ros::ServiceServer helloServ;

  /*!
   * \brief Publishes the timestamp of the last change of anything stored in the core, to e.g. tell a gui it should
   * refresh its data.
   */
  ros::Publisher lastChangedPub;

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle* nh;

  // Private member functions

  /*!
   * \brief changed is called every time something of the data stored in the core has changed.
   *
   * It then publishes the timestamp of the current so e.g. a connected gui gets notice of the changes and can update
   * its data
   * Also all connections are published on the config publisher so the devices reconfigure theirself if necessary and
   * nodes, who have been offline for while (e.g. connection lost, restart, ...) will get their connection information.
   * If DEBUG is defined, debugOutput is called now.
   * \see Tapi::TapiCore::configPub
   * \see Tapi::TapiCore::sendAllConnections
   * \see Tapi::TapiCore::debugOutput
   */
  void changed();

  /*! \brief Send a disconnect message for every device's feature and clear the maps of Tapi::TapiCore::connections and
   * Tapi::TapiCore::devices if a \c true is received on the subscriber.
   * \param cl The message waiting in the ros message queue where the data is stored. Has to be a \true to actually
   * clear all data.
   * \see clearAllSub
   */
  void clearAll(const std_msgs::Bool::ConstPtr& cl);

  /*!
   * \brief Deletes inactive devices and its connections
   *
   * If a \c true is received: Check's every device if its state is "inactive". If so it sends a disconnect message to
   * all other devices so they delete their connection to this inactive device. Then all data about this connections and
   * these inactive devices is deleted in the core.
   * To reconnect these inactive devices have to call the Hello-service again.
   * \param cl The message waiting in the ros message queue where the data is stored. Has to be a \true to actually
   * clear the data of inactive devices.
   * \see clearInactiveSub
   * \see Tapi::TapiCore::helloServ
   */
  void clearInactive(const std_msgs::Bool::ConstPtr& cl);

  /*!
   * \brief To sort devices there has to be a compare function, comparing their names.
   * \param first Pointer to the first device to compare
   * \param second Pointer to the second device to compare
   * \return \c true if the first device's name has to be above the second's device name in an alphabetically sort, \c
   * false when it's vice versa
   */
  static bool compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second);

  /*!
   * \brief Connect two features
   *
   * Connect two features (one subscriber to one publisher or one serviceclient to one serviceserver) of two devices.
   * The conection is stored in the core and then sent over the Config publisher
   * \param con The message waiting in the ros message queue where the data is stored, which features shall be
   * connected.
   * \see Tapi::TapiCore::configPub
   * \see \c Connect.msg in \c tapi_lib package
   */
  void connectFeatures(const tapi_lib::Connect::ConstPtr& con);

  /*!
   * \brief Print data stored in the core
   *
   * All devices, their data and their features are printed on the console.
   * \see Tapi::TapiCore::changed
   * \todo Print data of Tapi::TapiCore::connections as well.
   */
  void debugOutput();

  /*!
   * \brief Delete the connection of two features
   *
   * Deletes the connection of two features (one subscriber to one publisher or one serviceclient to one serviceserver).
   * Therefore the string inside the ros message (\c del parameter) is given to the overloaded
   * deleteConnection(std::string subscriberFeatureUUID) function.
   * \param del The message wainting in the ros message queue where the uuid of the feature is stored which shall be
   * disconnected.
   * \see Tapi::TapiCore::deleteConnection(std::string subscriberFeatureUUID)
   */
  void deleteConnection(const std_msgs::String::ConstPtr& del);

  /*!
   * \brief Delete the connection of two features
   *
   * Deletes the connection of two features (one subscriber to one publisher or one serviceclient to one serviceserver)
   * by sending out a disconnect message (publisher uuid = 0 and publisher feature uuid = 0) to both currently connected
   * devices over the Config publisher and then delete the stored data of the connection in the core.
   * \param subscriberFeatureUUID UUID of the feature on the subscriber/serviceclient which shall be disconnected
   * \see Tapi::TapiCore::configPub
   */
  void deleteConnection(std::string subscriberFeatureUUID);

  /*!
   * \brief Service to get a list of all currently connected features of devices
   *
   * If  the service is called with a \c true in the request, all conenctions stored in the Tapi::TapiCore::connections
   * map are copied into a \c vector of \c tapi_lib::Connection messages and this is sent as the response to the
   * requester.
   * \param listReq Request of the service call (has to be \c true to get a response)
   * \param listResp Response of the service call, including a \c vector of all connections.
   * \return \c true if the response has been generated and sent, \c false if the request was \c false.
   * \see \c Tapi::Connection in \c tapi_lib package
   * \see \c Connection.msg and \c GetConnectionList.srv in \c tapi_lib package
   */
  bool getConnectionList(tapi_lib::GetConnectionList::Request& listReq,
                         tapi_lib::GetConnectionList::Response& listResp);

  /*!
   * \brief Helper function to find and get the related device by a given uuid of a device-feature.
   * \param uuid UUID of the device-feature
   * \return Pointer pointing to the entry of the device in the Tapi::TapiCore::devices map. 0 if nothing is found.
   * \see \c Tapi::Device in \c tapi_lib package
   */
  Tapi::Device* getDeviceByFeatureUUID(std::string uuid);

  /*!
   * \brief Service to get a list of all devices currently connected to the core
   *
   * If the service is called with a \c true in the request, pointer to all devices stored in the
   * Tapi::TapiCore::devices map are created and stored in a \c vector.
   * Then they are sorted by their names and copied into a \c vector of \c tapi_lib::Device messages. This vector is
   * then sent as the response to the requester.
   * \param listReq Request of the service call (has to be \c true to get a response)
   * \param listResp Response of the service call, including a \c vector of all devices.
   * \return \c true if the response has been generated and sent, \c false if the request was \c false.
   * \see \c Tapi::Device in \c tapi_lib package
   * \see \c Device.msg and \c GetDeviceList.srv in \c tapi_lib package
   */
  bool getDevicesSorted(tapi_lib::GetDeviceList::Request& listReq, tapi_lib::GetDeviceList::Response& listResp);

  /*!
   * \brief Check if devices became inactive
   *
   * Called by the ros timer this functions checks if device has not sent a valid request to the Hello-service for a
   * period of time (2.5 times STANDARD_HEARTBEAT_INTERVAL).
   * If so the device gets the status "inactive" and no changes (e.g. new/deleted connections) can be applied to these
   * devices until they reconnect via the Hello-service
   * \param e Unused
   * \see heartbeatCheckTimer
   */
  void heartbeatCheck(const ros::TimerEvent& e);

  /*!
   * \brief Hello-service to connect devices to the core
   *
   * With the Hello-service devices can be connected to the core. This service has to be called regularly to keep the
   * state of the device as "alive".
   * The time after which devices shall send the next "hello" is defined by STANDARD_HEARTBEAT_INTERVAL. The description
   * of a correct Hello-message can be found in the tapi_lib package.
   * If a correct Hello message is received and the device is currently unknown, it is stored in the
   * Tapi::TapiCore::devices map.
   * If it is already a known member, its data and features are updated
   * \param helloReq Request with all data of the device (uuid, name, features, ...)
   * \param helloResp Response with a \c bool whether the connection/update was successful and the
   * STANDARD_HEARTBEAT_INTERVAL to tell the device, when it should send the next Hello-request.
   * \return \c true if the Hello message processing was successful, \c false if there was an error during processing
   * \see Tapi::TapiCore::helloServ
   * \see \c Hello.srv, \c Device.msg and \c Feature.msg in \c tapi_lib
   */
  bool hello(tapi_lib::Hello::Request& helloReq, tapi_lib::Hello::Response& helloResp);

  /*!
   * \brief Send all connetions over the Config publisher (which nodes shall subscribe to which other's topics)
   * \see configPub
   * \see \c Connection.msg in \c tapi_lib package
   */
  void sendAllConnections();
};
}

#endif  // TAPICORE_H
