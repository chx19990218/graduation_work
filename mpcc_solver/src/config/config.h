// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <ros/ros.h>

class Config {
 public:
  double theta_dot_upper_limit;
  double w_c;
  double w_l;
  double gamma;
  double w_u;
  double w_deltau;
  double mpcc_valid_dist;
  double angle_upper_limit;
  double obs_x_r;
  double obs_y_r;
  double Qobs;
  double end_gain_rate;
  double obs_x;
  double obs_y;
  double dp_length_rate;
  int N_SPLINE;
  bool obs_penalty_valid;
  bool generate_pcd_map_flag;
  bool enable_dp_flag;
  int ctrl_rate;
  int horizon;

  Config(const ros::NodeHandle& nh) {
    nh.getParam("theta_dot_upper_limit", theta_dot_upper_limit);
    nh.getParam("w_c", w_c);
    nh.getParam("w_l", w_l);
    nh.getParam("gamma", gamma);
    nh.getParam("w_u", w_u);
    nh.getParam("N_SPLINE", N_SPLINE);
    nh.getParam("mpcc_valid_dist", mpcc_valid_dist);
    nh.getParam("angle_upper_limit", angle_upper_limit);
    nh.getParam("obs_x_r", obs_x_r);
    nh.getParam("obs_y_r", obs_y_r);
    nh.getParam("Qobs", Qobs);
    nh.getParam("obs_penalty_valid", obs_penalty_valid);
    nh.getParam("w_deltau", w_deltau);
    nh.getParam("end_gain_rate", end_gain_rate);
    nh.getParam("obs_x", obs_x);
    nh.getParam("obs_y", obs_y);
    nh.getParam("generate_pcd_map_flag", generate_pcd_map_flag);
    nh.getParam("enable_dp_flag", enable_dp_flag);
    nh.getParam("dp_length_rate", dp_length_rate);
    nh.getParam("ctrl_rate", ctrl_rate);
    nh.getParam("horizon", horizon);
  }
};