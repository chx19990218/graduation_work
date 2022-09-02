// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include "map.h"
#include <eigen3/Eigen/Dense>

class Search {
 public:
  Search(){

  }
  std::vector<double> clutter_outer_x;
  std::vector<double> clutter_outer_y;
  std::vector<double> clutter_inner_x;
  std::vector<double> clutter_inner_y;
  void SphereSearch(const Map& map);
  int GetStage(const Map& map, double now_x, double now_y);
};