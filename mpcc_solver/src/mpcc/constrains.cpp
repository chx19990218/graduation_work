// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "map.h"

void GetStage(const Map& map) {
  int size = map.center_point_.size();
  std::vector<double> now_outer_point(2, 0.0);
  std::vector<double> now_inner_point(2, 0.0);
  std::vector<double> next_outer_point(2, 0.0);
  std::vector<double> next_inner_point(2, 0.0);
  for (int i = 0; i < size; i++) {
    now_outer_point[0] = map.outer_point_x_[i];
    now_outer_point[1] = map.outer_point_y_[i];
    now_inner_point[0] = map.inner_point_x_[i];
    now_inner_point[1] = map.inner_point_y_[i];
    next_outer_point[0] = map.outer_point_x_[i + 1];
    next_outer_point[1] = map.outer_point_y_[i + 1];
    next_inner_point[0] = map.inner_point_x_[i + 1];
    next_inner_point[1] = map.inner_point_y_[i + 1];
  }
  std::vector<double> direction{next_outer_point[0] - now_outer_point[0],
                                next_outer_point[1] - now_outer_point[1]};
  std::vector<double> now_from_inner_to_outer{
      now_outer_point[0] - now_inner_point[0],
      now_outer_point[1] - now_inner_point[1]};
  std::vector<double> next_from_inner_to_outer{
      next_outer_point[0] - next_inner_point[0],
      next_outer_point[1] - next_inner_point[1]};
  std::vector<std::vector<double>> rec;
  bool now_flag, next_flag;
  if (direction[0]*now_from_inner_to_outer[0] + direction[1]*now_from_inner_to_outer[1] < 0) {
    rec.emplace_back(now_outer_point);
    now_flag = true;
  } else {
    rec.emplace_back(now_inner_point);
    now_flag = false;
  }
  if (direction[0]*next_from_inner_to_outer[0] + direction[1]*next_from_inner_to_outer[1] < 0) {
    rec.emplace_back(now_inner_point);
    next_flag = true;
  } else {
    rec.emplace_back(now_outer_point);
    next_flag = false;
  }
  if (now_flag == next_flag) {
    // 一条线上
  } else {
    // 两条线上
  }
}

// 按顺时针传入
bool InRec(std::vector<std::vector<double>>& rec, double x, double y) {
  bool flag1 = ((rec[0][0] - x) * (rec[1][0] - rec[0][0]) +
                (rec[0][1] - y) * (rec[1][1] - rec[0][1])) < 0;
  bool flag2 = ((rec[1][0] - x) * (rec[2][0] - rec[1][0]) +
                (rec[1][1] - y) * (rec[2][1] - rec[1][1])) < 0;
  bool flag3 = ((rec[2][0] - x) * (rec[3][0] - rec[2][0]) +
                (rec[2][1] - y) * (rec[3][1] - rec[2][1])) < 0;
  bool flag4 = ((rec[3][0] - x) * (rec[0][0] - rec[3][0]) +
                (rec[3][1] - y) * (rec[0][1] - rec[3][1])) < 0;
  return flag1 & flag2 & flag3 & flag4;
}