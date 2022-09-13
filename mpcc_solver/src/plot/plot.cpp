// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "plot.h"

void Plot::plot(const Map& map, const Search& search, const Smooth& smooth,
                const Resample& resample, const Mpcc& mpcc) {
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

  // history
  Option option8(color_map[Blue], line_map[Seven]);
  plt::named_plot("horizon", mpcc.x_horizon, mpcc.y_horizon, option8.color_ + option8.line_);

  plt::legend();
  plt::show();
}