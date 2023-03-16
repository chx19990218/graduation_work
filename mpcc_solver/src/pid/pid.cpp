// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "controller.h"

void Pid::solve(Eigen::Vector3d des_p, Eigen::Vector3d des_v,
    const nav_msgs::Odometry& odom, const Config& config) {
  // p
  Eigen::Vector3d odom_p(odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         0.0);
  Eigen::Vector3d e_p = des_p - odom_p;
  Eigen::Vector3d u_p = kp * e_p + kd * (e_p - prev_e_p) * config.ctrl_rate;
  prev_e_p = e_p;

  // v
  Eigen::Vector3d odom_v(odom.twist.twist.linear.x,
                         odom.twist.twist.linear.y,
                         0.0);
  Eigen::Vector3d e_v = des_v + u_p - odom_v;
  Eigen::Vector3d u_v = kvp * e_v + kvd * (e_v - prev_e_v) * config.ctrl_rate;
  prev_e_v = e_v;
	
}