// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "search.h"

// 根据最小距离知道在哪个阶段,在里面更新了对应的边界
void Search::GetStage(const Map& map, double now_x, double now_y) {
  int point_size = map.center_point_.size();
  // 常规不能用距离最小值确定阶段，很可能其他点距离最小，比如掉头工况
  // 和下一个点做距离比较
  if (pre_index == -1) {
    Eigen::ArrayXd diff_x = map.center_point_x_.array() - now_x;
    Eigen::ArrayXd diff_y = map.center_point_y_.array() - now_y;
    Eigen::ArrayXd dist_square = diff_x.square() + diff_y.square();
    std::vector<double> dist_square_vec(
        dist_square.data(), dist_square.data() + dist_square.size());
    auto min_iter =
        std::min_element(dist_square_vec.begin(), dist_square_vec.end());
    now_index = min_iter - dist_square_vec.begin();
  } else {
    int candidate_index;
    if (pre_index == point_size - 1) {
      candidate_index = 0;
    } else {
      candidate_index = pre_index + 1;
    }
    double dist_now = std::pow(map.center_point_x_[pre_index] - now_x, 2) +
                      std::pow(map.center_point_y_[pre_index] - now_y, 2);
    double dist_next = std::pow(map.center_point_x_[candidate_index] - now_x, 2) +
                       std::pow(map.center_point_y_[candidate_index] - now_y, 2);
    if (dist_next < dist_now) {
      now_index = candidate_index;
    }
  }
  if (now_index != pre_index) {
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
    ResampleBorder(map, last_index, now_index, next_index);
  }
  pre_index = now_index;
}

void Search::ResampleBorder(const Map& map, int last_index, int now_index,
                            int next_index) {
  clutter_outer_x.clear();
  clutter_outer_y.clear();
  clutter_inner_x.clear();
  clutter_inner_y.clear();
  // 对last-next边界密集采样
  double sample_interval = 0.05;
  // 外侧前面一段
  double outer_former_stage_num =
      std::sqrt(std::pow(map.outer_point_x_[last_index] -
                             map.outer_point_x_[now_index],
                         2) +
                std::pow(map.outer_point_y_[last_index] -
                             map.outer_point_y_[now_index],
                         2)) /
      sample_interval;
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
  double outer_latter_stage_num =
      std::sqrt(std::pow(map.outer_point_x_[next_index] -
                             map.outer_point_x_[now_index],
                         2) +
                std::pow(map.outer_point_y_[next_index] -
                             map.outer_point_y_[now_index],
                         2)) /
      sample_interval;
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
  double inner_former_stage_num =
      std::sqrt(std::pow(map.inner_point_x_[last_index] -
                             map.inner_point_x_[now_index],
                         2) +
                std::pow(map.inner_point_y_[last_index] -
                             map.inner_point_y_[now_index],
                         2)) /
      sample_interval;
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
  double inner_latter_stage_num =
      std::sqrt(std::pow(map.inner_point_x_[next_index] -
                             map.inner_point_x_[now_index],
                         2) +
                std::pow(map.inner_point_y_[next_index] -
                             map.inner_point_y_[now_index],
                         2)) /
      sample_interval;
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

void Search::SphereSearch(const Map& map) {
  // 设置起始位置
  double now_x = 0.5, now_y = 0.5;
  // 设置搜索方向
  Eigen::Vector2d search_direction(0.0, 1.0);

  GetStage(map, now_x, now_y);
  double sphere_r = SphereExpansion(now_x, now_y);
  while (!ReachTarget(map, now_x, now_y, sphere_r)) {
    if (!TargetInCircle(map, now_x, now_y, sphere_r)) {
      no_target_cnt_++;
    } else {
      no_target_cnt_ = 0;
    }

    GetStage(map, now_x, now_y);
    std::vector<std::vector<double>> sample_result;
    SphereSample(now_x, now_y, sphere_r, search_direction, sample_result);
    std::vector<double> next_pos = std::vector<double>(2);
    
    // 顺道更新了下步的半径
    DecideNextPos(sample_result, next_pos, sphere_r);
    // 顺道更新了下步的位置
    ReviseSearchDirection(now_x, now_y, next_pos, search_direction);

    result_x.emplace_back(now_x);
    result_y.emplace_back(now_y);
  }
}

void Search::ReviseSearchDirection(double& now_x, double& now_y,
                                   const std::vector<double>& next_pos,
                                   Eigen::Vector2d& search_direction) {
  Eigen::Vector2d next_direction(next_pos[0] - now_x, next_pos[1] - now_y);
  next_direction /= next_direction.norm();
  search_direction[0] = next_direction[0];
  search_direction[1] = next_direction[1];
  // 顺道更新下当前位置
  now_x = next_pos[0];
  now_y = next_pos[1];
}

void Search::DecideNextPos(
    const std::vector<std::vector<double>>& sample_result,
    std::vector<double>& next_pos, double& sphere_r) {
  next_pos.clear();
  double r;
  for (int i = 0; i < sample_result.size(); i++) {
    if (i == 0) {
      r = SphereExpansion(sample_result[i][0], sample_result[i][1]);
      next_pos[0] = sample_result[i][0];
      next_pos[1] = sample_result[i][1];
    } else {
      double temp = SphereExpansion(sample_result[i][0], sample_result[i][1]);
      if (temp > r) {
        r = temp;
        next_pos[0] = sample_result[i][0];
        next_pos[1] = sample_result[i][1];
      }
    }
  }
  // 已经膨胀过了，直接在这更新
  sphere_r = r;
}

void Search::SphereSample(double now_x, double now_y, double r,
                          const Eigen::Vector2d& search_direction,
                          std::vector<std::vector<double>>& sample_result) {
  sample_result.clear();
  double deg_interval = 30.0 / 180.0 * PI;
  // 把两边排除
  int sample_num = static_cast<int>(PI / 2.0 / deg_interval - 1.0);
  double sample_x, sample_y;
  for (int i = 0; i <= sample_num; i++) {
    if (i == 0) {
      sample_x = search_direction[0] * r + now_x;
      sample_y = search_direction[1] * r + now_y;
      sample_result.emplace_back(std::vector<double>{sample_x, sample_y});
    } else {
      double theta = deg_interval * static_cast<int>(i);
      // 逆时针
      double new_direction_x = search_direction[0] * std::cos(theta) -
                               search_direction[1] * std::sin(theta);
      double new_direction_y = search_direction[0] * std::sin(theta) +
                               search_direction[1] * std::cos(theta);
      sample_x = new_direction_x * r + now_x;
      sample_y = new_direction_y * r + now_y;
      sample_result.emplace_back(std::vector<double>{sample_x, sample_y});
      //顺时针
      new_direction_x = search_direction[0] * std::cos(theta) +
                        search_direction[1] * std::sin(theta);
      new_direction_y = -search_direction[0] * std::sin(theta) +
                        search_direction[1] * std::cos(theta);
      sample_x = new_direction_x * r + now_x;
      sample_y = new_direction_y * r + now_y;
      sample_result.emplace_back(std::vector<double>{sample_x, sample_y});
    }
  }
}

// 在stage做球体膨胀返回膨胀半径
double Search::SphereExpansion(double now_x, double now_y) {
  Eigen::VectorXd outer_x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      clutter_outer_x.data(), clutter_outer_x.size());
  Eigen::VectorXd outer_y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      clutter_outer_y.data(), clutter_outer_y.size());
  Eigen::VectorXd inner_x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      clutter_inner_x.data(), clutter_inner_x.size());
  Eigen::VectorXd inner_y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      clutter_inner_y.data(), clutter_inner_y.size());

  Eigen::ArrayXd diff_outer_x = outer_x.array() - now_x;
  Eigen::ArrayXd diff_outer_y = outer_y.array() - now_y;
  Eigen::ArrayXd outer_square = diff_outer_x.square() + diff_outer_y.square();
  std::vector<double> outer_square_vec(
      outer_square.data(), outer_square.data() + outer_square.size());
  double outer_min_dist = std::sqrt(
      *std::min_element(outer_square_vec.begin(), outer_square_vec.end()));

  Eigen::ArrayXd diff_inner_x = inner_x.array() - now_x;
  Eigen::ArrayXd diff_inner_y = inner_y.array() - now_y;
  Eigen::ArrayXd inner_square = diff_inner_x.square() + diff_inner_y.square();
  std::vector<double> inner_square_vec(
      inner_square.data(), inner_square.data() + inner_square.size());
  double inner_min_dist = std::sqrt(
      *std::min_element(inner_square_vec.begin(), inner_square_vec.end()));

  return std::min(outer_min_dist, inner_min_dist);
}

bool Search::ReachTarget(const Map& map, double now_x, double now_y, double r) {
  bool target_in_circle_flag = TargetInCircle(map, now_x, now_y, r);
  if (no_target_cnt_ > 5 && target_in_circle_flag) {
    return true;
  } else {
    return false;
  }
}

bool Search::TargetInCircle(const Map& map, double now_x, double now_y,
                            double r) {
  double dist = std::sqrt(pow(map.center_point_x_[0] - now_x, 2) +
                          pow(map.center_point_y_[0] - now_y, 2));
  return dist < r;
}