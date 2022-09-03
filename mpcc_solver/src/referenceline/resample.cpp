// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "resample.h"

void Resample::FitResample(const Smooth& smooth) {
  auto vec_x = smooth.result_x;
  auto vec_y = smooth.result_y;
  Eigen::VectorXd X = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec_x.data(), smooth.result_x.size());
  Eigen::VectorXd Y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec_y.data(), smooth.result_y.size());
  spline.gen2DSpline(X, Y);
}