// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "search.h"

int Search::GetStage(const Map& map, double now_x, double now_y) {
  Eigen::ArrayXd diff_x = map.center_point_x_.array() - now_x;
  Eigen::ArrayXd diff_y = map.center_point_y_.array() - now_y;
  Eigen::ArrayXd dist_square = diff_x.square() + diff_y.square();
  std::vector<double> dist_square_vec(dist_square.data(),
                                      dist_square.data() + dist_square.size());
  auto min_iter =
      std::min_element(dist_square_vec.begin(), dist_square_vec.end());

  int now_index = min_iter - dist_square_vec.begin();
  return now_index;
}

void Search::SphereSearch(const Map& map) {
  int point_size = map.center_point_.size();
  double now_x = 0.5, now_y = 0.5;
  int now_index = GetStage(map, now_x, now_y);
  int last_index, next_index;
  if (now_index == 0) {
    last_index = point_size - 1;
  } else {
    last_index = now_index - 1;
  }
  if (now_index == point_size - 1) {
    next_index = 0;
  } else {
    next_index = now_index + 1;
  }

  // 对last-next边界密集采样
  double sample_interval = 0.05;
  // 外侧前面一段
  double outer_former_stage_num = std::sqrt(std::pow(map.outer_point_x_[last_index] -
                                                  map.outer_point_x_[now_index],
                                              2) +
                                     std::pow(map.outer_point_y_[last_index] -
                                                  map.outer_point_y_[now_index],
                                              2)) / sample_interval;
  double outer_former_stage_x_interval =
      (map.outer_point_x_[now_index] - map.outer_point_x_[last_index]) /
      outer_former_stage_num;
  double outer_former_stage_y_interval =
      (map.outer_point_y_[now_index] - map.outer_point_y_[last_index]) /
      outer_former_stage_num;
  for (int i = 0; i <= outer_former_stage_num; i++) {
    double outer_former_stage_x =
        map.outer_point_x_[last_index] + i * outer_former_stage_x_interval;
    double outer_former_stage_y =
        map.outer_point_y_[last_index] + i * outer_former_stage_y_interval;
    clutter_outer_x.emplace_back(outer_former_stage_x);
    clutter_outer_y.emplace_back(outer_former_stage_y);
  }
  // 外侧后面一段
  double outer_latter_stage_num = std::sqrt(std::pow(map.outer_point_x_[next_index] -
                                                  map.outer_point_x_[now_index],
                                              2) +
                                     std::pow(map.outer_point_y_[next_index] -
                                                  map.outer_point_y_[now_index],
                                              2)) / sample_interval;
  double outer_latter_stage_x_interval =
      (map.outer_point_x_[next_index] - map.outer_point_x_[now_index]) /
      outer_latter_stage_num;
  double outer_latter_stage_y_interval =
      (map.outer_point_y_[next_index] - map.outer_point_y_[now_index]) /
      outer_latter_stage_num;
  for (int i = 0; i <= outer_latter_stage_num; i++) {
    double outer_latter_stage_x =
        map.outer_point_x_[now_index] + i * outer_latter_stage_x_interval;
    double outer_latter_stage_y =
        map.outer_point_y_[now_index] + i * outer_latter_stage_y_interval;
    clutter_outer_x.emplace_back(outer_latter_stage_x);
    clutter_outer_y.emplace_back(outer_latter_stage_y);
  }
  // 里侧前面一段
  double inner_former_stage_num = std::sqrt(std::pow(map.inner_point_x_[last_index] -
                                                  map.inner_point_x_[now_index],
                                              2) +
                                     std::pow(map.inner_point_y_[last_index] -
                                                  map.inner_point_y_[now_index],
                                              2)) / sample_interval;
  double inner_former_stage_x_interval =
      (map.inner_point_x_[now_index] - map.inner_point_x_[last_index]) /
      inner_former_stage_num;
  double inner_former_stage_y_interval =
      (map.inner_point_y_[now_index] - map.inner_point_y_[last_index]) /
      inner_former_stage_num;
  for (int i = 0; i <= inner_former_stage_num; i++) {
    double inner_former_stage_x =
        map.inner_point_x_[last_index] + i * inner_former_stage_x_interval;
    double inner_former_stage_y =
        map.inner_point_y_[last_index] + i * inner_former_stage_y_interval;
    clutter_inner_x.emplace_back(inner_former_stage_x);
    clutter_inner_y.emplace_back(inner_former_stage_y);
  }
  // 里侧后面一段
  double inner_latter_stage_num = std::sqrt(std::pow(map.inner_point_x_[next_index] -
                                                  map.inner_point_x_[now_index],
                                              2) +
                                     std::pow(map.inner_point_y_[next_index] -
                                                  map.inner_point_y_[now_index],
                                              2)) / sample_interval;
  double inner_latter_stage_x_interval =
      (map.inner_point_x_[next_index] - map.inner_point_x_[now_index]) /
      inner_latter_stage_num;
  double inner_latter_stage_y_interval =
      (map.inner_point_y_[next_index] - map.inner_point_y_[now_index]) /
      inner_latter_stage_num;
  for (int i = 0; i <= inner_latter_stage_num; i++) {
    double inner_latter_stage_x =
        map.inner_point_x_[now_index] + i * inner_latter_stage_x_interval;
    double inner_latter_stage_y =
        map.inner_point_y_[now_index] + i * inner_latter_stage_y_interval;
    clutter_inner_x.emplace_back(inner_latter_stage_x);
    clutter_inner_y.emplace_back(inner_latter_stage_y);
  }
  
}