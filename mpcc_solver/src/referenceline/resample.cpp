// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "resample.h"

void Resample::FitResample(const Smooth& smooth) {
  auto vec_x = smooth.result_x;
  auto vec_y = smooth.result_y;
  Eigen::VectorXd X = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec_x.data(), smooth.result_x.size());
  Eigen::VectorXd Y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec_y.data(), smooth.result_y.size());
  spline.gen2DSpline(X, Y);

  // 根据中心线向外拓展半径
  // std::vector<double> center_s(spline.path_data_.s.data(), spline.path_data_.s.data() + spline.path_data_.s.size());
  // for (auto s : center_s) {
    // auto center_p = spline.getPostion(s);
    // auto center_v = spline.getDerivative(s);
    // double r = 0.5;
    // double x1 = center_p(0) + r * (-center_v(1));
    // double y1 = center_p(1) + r * (center_v(0));
  //   x.emplace_back(x1);
  //   y.emplace_back(y1);
  // }
}