// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <eigen3/Eigen/Dense>

#include "map.h"

class Search {
 public:
  Search() {}
  int no_target_cnt_ = 0;
  int pre_index = -1;
  int now_index = 0;
  std::vector<double> clutter_outer_x;
  std::vector<double> clutter_outer_y;
  std::vector<double> clutter_inner_x;
  std::vector<double> clutter_inner_y;
  std::vector<double> result_x;
  std::vector<double> result_y;
  void SphereSearch(const Map& map);
  void GetStage(const Map& map, double now_x, double now_y);
  bool ReachTarget(const Map& map, double now_x, double now_y, double r);
  bool TargetInCircle(const Map& map, double now_x, double now_y, double r);
  void ResampleBorder(const Map& map, int last_index, int now_index,
                      int next_index);
  double SphereExpansion(double now_x, double now_y);
  void SphereSample(double now_x, double now_y, double r,
                    const Eigen::Vector2d& search_direction,
                    std::vector<std::vector<double>>& sample_result);
  void DecideNextPos(const std::vector<std::vector<double>>& sample_result,
                     std::vector<double>& next_pos, double& sphere_r);
  void ReviseSearchDirection(double& now_x, double& now_y,
                             const std::vector<double>& next_pos,
                             Eigen::Vector2d& search_direction);
};