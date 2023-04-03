// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "log.h"

Log::Log(const Map& map) {
  std::string package_path = ros::package::getPath("mpcc_solver");
  // 向上找两级
  std::size_t last_index = package_path.rfind("/");
  package_path = package_path.substr(0, last_index);
  last_index = package_path.rfind("/");
  package_path = package_path.substr(0, last_index + 1);
  log_path = package_path + "log";
  // 创建文件夹
  if (access(log_path.c_str(), 0) == -1) {
    int ret = mkdir(log_path.c_str(), 0755);
  }

  x_log.open(log_path + "/x.txt", std::ios::out);
  x_log << "des_p/odom_p/des_v/odom_v/des_a" << std::endl;
  y_log.open(log_path + "/y.txt", std::ios::out);
  y_log << "des_p/odom_p/des_v/odom_v/des_a" << std::endl;
  z_log.open(log_path + "/z.txt", std::ios::out);
  z_log << "des_p/odom_p/des_v/odom_v/des_a" << std::endl;
  // t_log << 
  corridor_log.open(log_path + "/corridor.txt", std::ios::out);
  for (int i = 0; i < map.center_point_x_.size(); i++) {
    corridor_log << map.center_point_x_[i] << "/" << map.center_point_y_[i] << "/"
      << map.outer_point_x_[i] << "/" << map.outer_point_y_[i] << "/"
      << map.inner_point_x_[i] << "/" << map.inner_point_y_[i] << std::endl;
  }
}

Log::~Log() {
  x_log.close();
  y_log.close();
  z_log.close();
  corridor_log.close();
};

void Log::Record(const quadrotor_msgs::PositionCommand& cmdMsg,
  const nav_msgs::Odometry& odom) {
  x_log << cmdMsg.position.x << "/" << odom.pose.pose.position.x << "/"
        << cmdMsg.velocity.x << "/" << odom.twist.twist.linear.x << "/"
        << cmdMsg.acceleration.x << std::endl;
  y_log << cmdMsg.position.y << "/" << odom.pose.pose.position.y << "/"
        << cmdMsg.velocity.y << "/" << odom.twist.twist.linear.y << "/"
        << cmdMsg.acceleration.y << std::endl;
  z_log << cmdMsg.position.z << "/" << odom.pose.pose.position.z << "/"
        << cmdMsg.velocity.z << "/" << odom.twist.twist.linear.z << "/"
        << cmdMsg.acceleration.z << std::endl;
}