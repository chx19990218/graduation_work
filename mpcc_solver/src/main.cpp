#include <ros/ros.h>
#include "config.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpcc_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  ros::Rate rate(100);

  State state;
  Input input;

  // while (ros::ok()) {
  //   ros::spinOnce();
  //   mission.process();
  //   rate.sleep();
  // }
  return 0;
}