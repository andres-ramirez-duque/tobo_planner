#include "ros/ros.h"
#include "tobo_planner/SensorValue.h"
#include <string>
#include <stdlib.h>
#include <time.h>       /* time */

bool readsensor(tobo_planner::SensorValue::Request  &req,
         tobo_planner::SensorValue::Response &res)
{
  std::string sensor_msg("Sensing engagement");
  res.sensor_value = true;
  res.message = sensor_msg;
  ROS_INFO("request: %s", req.request_type.c_str());
  ROS_INFO("sending back response: [%d]", res.sensor_value);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensortransition_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_sensor_value", readsensor);
  ROS_INFO("Sensing");
  ros::spin();

  return 0;
}
