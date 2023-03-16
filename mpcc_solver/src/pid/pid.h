// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include "mpcc.h"
#include "config.h"

class Pid {
 public:
  Eigen::Vector3d prev_e_p, prev_e_v;
  void solve(Eigen::Vector3d des_p, Eigen::Vector3d des_v,
    const nav_msgs::Odometry& odom, const Config& config);
};