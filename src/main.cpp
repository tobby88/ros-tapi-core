#include "tapicore.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi");
  ros::NodeHandle nh;
  Tapi::TapiCore tapi(&nh);

  while (ros::ok())
    ros::spin();

  return 0;
}
