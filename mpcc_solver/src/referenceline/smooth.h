// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <eigen3/Eigen/Dense>
#include "search.h"

class Smooth {
 public:
  Eigen::MatrixXd matrix_a_smooth_;
  Eigen::MatrixXd matrix_a_length_;
  Eigen::MatrixXd matrix_a_similarity_;
  std::vector<double> result_x;
  std::vector<double> result_y;
  void Fem(const Search& search);
};