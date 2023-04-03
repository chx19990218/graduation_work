// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "map.h"

class Log {
 public:
  std::string log_path;

  std::ofstream x_log;
  std::ofstream y_log;
  std::ofstream z_log;

  std::ofstream corridor_log;

  Log(const Map& map);
  ~Log();
  void Record(const quadrotor_msgs::PositionCommand& cmdMsg, const nav_msgs::Odometry& odom);
  // Parameter_t& param;
  // std::stringstream create_time;
  // ofstream flightlog;
  // string folderPath;

  // Logger(Parameter_t&);
  // void close_log();
  // void record_log(const ros::Duration& now_time,const Desired_State_t& des,const Odom_Data_t& odom_data, 
  //                 const Controller_Output_t& u,const RC_Data_t& rc_data,const Imu_Data_t& imu_data,const double& hovper, const Eso_t& eso_data);

};