#include "ros/ros.h"
#include "tobo_planner/SensorTransition.h"
#include <string>
#include <stdlib.h>
#include <time.h>       /* time */

bool comparesensor(tobo_planner::SensorTransition::Request  &req,
         tobo_planner::SensorTransition::Response &res)
{
  int choice = 0;
  srand (time(NULL));
  std::string sensor ("[4] sensehighanxiety ");
  
  if (sensor.compare(req.sensor_transition) == 0){
    choice = rand() % 2;
  }
  res.sensor_state = choice;
  ROS_INFO("request: %s", req.sensor_transition.c_str());
  ROS_INFO("sending back response: [%ld]", (long int)res.sensor_state);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensortransition_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("sensortransition", comparesensor);
  ROS_INFO("Sensing");
  ros::spin();

  return 0;
}
