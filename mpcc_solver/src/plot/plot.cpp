// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "plot.h"

void Plot::plot(const Map& map, const Search& search, const Smooth& smooth,
                const Resample& resample, const Mpcc& mpcc, const Obstacle& obstacle) {
  plt::figure(1);

  // 中心线
  Option option1(color_map[Yellow], line_map[Two]);
  std::vector<double> xc(
      map.center_point_x_.data(),
      map.center_point_x_.data() + map.center_point_x_.size());
  std::vector<double> yc(
      map.center_point_y_.data(),
      map.center_point_y_.data() + map.center_point_y_.size());
  plt::plot(xc, yc, option1.color_ + option1.line_);

  // 外侧线
  Option option2(color_map[Black], line_map[One]);
  std::vector<double> xo(map.outer_point_x_.data(),
                         map.outer_point_x_.data() + map.outer_point_x_.size());
  std::vector<double> yo(map.outer_point_y_.data(),
                         map.outer_point_y_.data() + map.outer_point_y_.size());
  plt::plot(xo, yo, option2.color_ + option2.line_);

  // 内侧线
  Option option3(color_map[Black], line_map[One]);
  std::vector<double> xi(map.inner_point_x_.data(),
                         map.inner_point_x_.data() + map.inner_point_x_.size());
  std::vector<double> yi(map.inner_point_y_.data(),
                         map.inner_point_y_.data() + map.inner_point_y_.size());
  plt::plot(xi, yi, option3.color_ + option3.line_);

  // 搜索
  Option option4(color_map[Green], line_map[Eight]);
  plt::named_plot("search", search.result_x, search.result_y,
                  option4.color_ + option4.line_);

  // 平滑
  Option option5(color_map[Cyan], line_map[Nine]);
  plt::named_plot("smooth", smooth.result_x, smooth.result_y, option5.color_ + option5.line_);

  // 拟合 重采样
  Option option6(color_map[Magenta], line_map[Six]);
  std::vector<double> x_resample(resample.spline.path_data_.X.data(),
                                 resample.spline.path_data_.X.data() +
                                     resample.spline.path_data_.X.size());
  std::vector<double> y_resample(resample.spline.path_data_.Y.data(),
                                 resample.spline.path_data_.Y.data() +
                                     resample.spline.path_data_.Y.size());
  plt::named_plot("fit and resample", x_resample, y_resample, option6.color_ + option6.line_);

  // history
  Option option7(color_map[Blue], line_map[One]);
  plt::named_plot("history", mpcc.x_history, mpcc.y_history, option7.color_ + option7.line_);

  // horizon
  Option option8(color_map[Blue], line_map[Seven]);
  plt::named_plot("horizon", mpcc.x_horizon, mpcc.y_horizon, option8.color_ + option8.line_);

  // theta horizon
  Option option9(color_map[Red], line_map[Seven]);
  plt::plot(mpcc.theta_x_, mpcc.theta_y_, option9.color_ + option9.line_);

  // obstacle
  std::vector<double> x1, y1;
  for (int i = 0; i < obstacle.obstacle_pos_.size(); i++) {
    x1.emplace_back(obstacle.obstacle_pos_[i][0]);
    y1.emplace_back(obstacle.obstacle_pos_[i][1]);
  }
  plt::fill(x1, y1, {});

  // // 动态规划最优轨迹
  // Option option10(color_map[Cyan], line_map[Ten]);
  // plt::plot(mpcc.optimal_path_x, mpcc.optimal_path_y, option10.color_ + option10.line_);

  // 撒点
  Option option11(color_map[Green], line_map[Six]);
  for (int i = 0; i < obstacle.grid_x_.size(); i++) {
    plt::plot(obstacle.grid_x_[i], obstacle.grid_y_[i], option11.color_ + option11.line_);
  }

  // 扩张边界
  Option option12(color_map[Magenta], line_map[Three]);
  plt::plot(mpcc.left_border_x, mpcc.left_border_y, option12.color_ + option12.line_);
  plt::plot(mpcc.right_border_x, mpcc.right_border_y, option12.color_ + option12.line_);
  

  // std::vector<double> x, y;
  // for(int i=0;i<mpcc.horizon;i++){
  //   x.emplace_back(mpcc.stage[i].state[0]);
  //   y.emplace_back(mpcc.stage[i].state[2]);
  // }
  // Option option10(color_map[Yellow], line_map[Seven]);
  // plt::plot(x, y, option10.color_ + option10.line_);

  plt::legend();
  plt::show();
}