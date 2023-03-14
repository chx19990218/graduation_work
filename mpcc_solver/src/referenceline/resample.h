// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <eigen3/Eigen/Dense>

#include "smooth.h"

class Resample {
 public:
  ArcLengthSpline spline;
  void FitResample(const Smooth& smooth);
  std::vector<double> x;
  std::vector<double> y;
};