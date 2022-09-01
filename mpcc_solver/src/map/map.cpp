// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "map.h"



void Map::GenerateMap() {
  int point_size = center_point_.size();
  // 顺时针
  // 点是拐点 
  center_point_x_ = Eigen::VectorXd::Zero(point_size + 1);
  center_point_y_ = Eigen::VectorXd::Zero(point_size + 1);
  outer_point_x_ = Eigen::VectorXd::Zero(point_size + 1);
  outer_point_y_ = Eigen::VectorXd::Zero(point_size + 1);
  inner_point_x_ = Eigen::VectorXd::Zero(point_size + 1);
  inner_point_y_ = Eigen::VectorXd::Zero(point_size + 1);
  std::vector<double> last_point, now_point, next_point;
  for (int i = 0; i < point_size; i++) {
    center_point_x_[i] = center_point_[i][0];
    center_point_y_[i] = center_point_[i][1];

    if (i == 0) {
      last_point = std::vector<double>{center_point_[point_size - 1][0],
                                center_point_[point_size - 1][1]};
    } else {
      last_point = std::vector<double>{center_point_[i - 1][0], 
                                center_point_[i - 1][1]};
    }
    now_point = std::vector<double>{center_point_[i][0], center_point_[i][1]};
    if (i == point_size - 1) {
      next_point = std::vector<double>{center_point_[0][0],
                                       center_point_[0][1]};
    } else {
      next_point = std::vector<double>{center_point_[i + 1][0], 
                                       center_point_[i + 1][1]};
    }
    auto cornertype = GetCornerType(last_point, now_point, next_point);

    Eigen::Vector2d norm_vec_now_to_last(last_point[0] - now_point[0],
                                         last_point[1] - now_point[1]);
    norm_vec_now_to_last /= norm_vec_now_to_last.norm();
    norm_vec_now_to_last *= width/2;
    Eigen::Vector2d norm_vec_now_to_next(next_point[0] - now_point[0],
                                         next_point[1] - now_point[1]);
    norm_vec_now_to_next /= norm_vec_now_to_next.norm();
    norm_vec_now_to_next *= width/2;

    auto vec_sum = norm_vec_now_to_last + norm_vec_now_to_next;
    if (cornertype == CornerType::Outer) {
      outer_point_x_[i] = now_point[0] - vec_sum[0];
      outer_point_y_[i] = now_point[1] - vec_sum[1];
      inner_point_x_[i] = now_point[0] + vec_sum[0];
      inner_point_y_[i] = now_point[1] + vec_sum[1];
    } else {
      outer_point_x_[i] = now_point[0] + vec_sum[0];
      outer_point_y_[i] = now_point[1] + vec_sum[1];
      inner_point_x_[i] = now_point[0] - vec_sum[0];
      inner_point_y_[i] = now_point[1] - vec_sum[1];
    } 
  }
  center_point_x_[center_point_.size()] = center_point_[0][0];
  center_point_y_[center_point_.size()] = center_point_[0][1];
  outer_point_x_[center_point_.size()] = outer_point_x_[0];
  outer_point_y_[center_point_.size()] = outer_point_y_[0];
  inner_point_x_[center_point_.size()] = inner_point_x_[0];
  inner_point_y_[center_point_.size()] = inner_point_y_[0];
  centerline.gen2DSpline(center_point_x_, center_point_y_);
}
// 拐角分两类:外角/内角
// 外角：上右下左顺序构成的
// 内角：右上左下顺序构成的
// 当前点与前后点构成向量的和Vsum
// inner: 外角Vsum方向，内角Vsum反向
// outer: 外角Vsum反向，内角Vsum方向
CornerType Map::GetCornerType(std::vector<double>& last_point,
                              std::vector<double>& now_point,
                              std::vector<double>& next_point) {
  Eigen::Vector2d former_vec(now_point[0] - last_point[0],
                             now_point[1] - last_point[1]);
  Eigen::Vector2d latter_vec(next_point[0] - now_point[0],
                             next_point[1] - now_point[1]);
  VecType former_vec_type = GetVecType(former_vec);
  VecType latter_vec_type = GetVecType(latter_vec);
  
  if ((former_vec_type == VecType::YPos && latter_vec_type == VecType::XPos) ||
      (former_vec_type == VecType::XPos && latter_vec_type == VecType::YNeg) ||
      (former_vec_type == VecType::YNeg && latter_vec_type == VecType::XNeg) ||
      (former_vec_type == VecType::XNeg && latter_vec_type == VecType::YPos)) {
    return CornerType::Outer;
  } else {
    return CornerType::Inner;
  }
}
VecType Map::GetVecType(Eigen::Vector2d& vec) {
  Eigen::Vector2d vec_i(1.0, 0.0);
  Eigen::Vector2d vec_j(0.0, 1.0);
  double deg_i = acos(vec.dot(vec_i)/vec.norm());
  double deg_j = acos(vec.dot(vec_j)/vec.norm());
  double axis_tolerance = 20.0/180.0 * PI;
  if (std::abs(deg_i - 0.0) < axis_tolerance) {
    return VecType::XPos;
  } else if (std::abs(deg_i - PI) < axis_tolerance) {
    return VecType::XNeg;
  } else if (std::abs(deg_j - 0.0) < axis_tolerance) {
    return VecType::YPos;
  } else if (std::abs(deg_j - PI) < axis_tolerance) {
    return VecType::YNeg;
  } else if (deg_i < PI/2 && deg_j < PI/2) {
    return VecType::FirstPhase;
  } else if (deg_i > PI/2 && deg_j < PI/2) {
    return VecType::SecondPhase;
  } else if (deg_i > PI/2 && deg_j > PI/2) {
    return VecType::ThirdPhase;
  } else if (deg_i < PI/2 && deg_j > PI/2) {
    return VecType::FourthPhase;
  } else {
    return VecType::Error;
  }
}