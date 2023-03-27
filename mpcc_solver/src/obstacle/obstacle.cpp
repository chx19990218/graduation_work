// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "obstacle.h"

void Obstacle::Update(const Resample& referenceline, const Map& map,
                      Mpcc& mpcc, Eigen::SparseMatrix<double> state, const Config& config) {
  obstacle_pos_ = mpcc.obstacle_pos_;
  GenerateGridCoordinate(referenceline, map, mpcc, config);
  DPForward(mpcc, referenceline, state);
  DPBackward(mpcc);
  ExpandPath(mpcc);
}

void Obstacle::GenerateGridCoordinate(const Resample& referenceline,
                                      const Map& map, Mpcc& mpcc, const Config& config) {
  double obs_x = (obstacle_pos_[0][0] + obstacle_pos_[2][0]) / 2.0;
  double obs_y = (obstacle_pos_[0][1] + obstacle_pos_[2][1]) / 2.0;
  double obs_theta = referenceline.spline.porjectOnSpline(obs_x, obs_y);
  double horizon_dist = mpcc.horizon * config.theta_dot_upper_limit * mpcc.Ts + 0.1;
  // 投影求size
  double obs_point_theta1 = referenceline.spline.porjectOnSpline(obstacle_pos_[0][0], obstacle_pos_[0][1]);
  double obs_point_theta2 = referenceline.spline.porjectOnSpline(obstacle_pos_[1][0], obstacle_pos_[1][1]);
  double obs_point_theta3 = referenceline.spline.porjectOnSpline(obstacle_pos_[2][0], obstacle_pos_[2][1]);
  double obs_point_theta4 = referenceline.spline.porjectOnSpline(obstacle_pos_[3][0], obstacle_pos_[3][1]);
  double obs_size = std::max({obs_point_theta1, obs_point_theta2, obs_point_theta3, obs_point_theta4}) -
    std::min({obs_point_theta1, obs_point_theta2, obs_point_theta3, obs_point_theta4});
  double obs_start_theta = obs_theta - obs_size / 2.0;
  double obs_end_theta = obs_theta + obs_size / 2.0;
  // 用 rate * horizon_dist作为dp范围
  double start_theta = std::max(obs_theta -
    std::max(horizon_dist * config.dp_length_rate, obs_size), 0.0);
  double end_theta = obs_theta + horizon_dist * config.dp_length_rate;
  double interval = (end_theta - start_theta) / static_cast<double>(row_size);
  // std::cout << start_theta << "," << obs_theta << "," << end_theta << std::endl;
  
  double theta = obs_theta;
  std::vector<double> former_theta;
  double dynamic_interval = interval;
  while (theta > start_theta) {
    former_theta.emplace_back(theta);
    theta -= dynamic_interval;
    dynamic_interval = DynamicInterval(theta, interval,
      start_theta, end_theta, obs_start_theta, obs_end_theta);
  }
  std::reverse(former_theta.begin(), former_theta.end());
  std::vector<double> later_theta;
  theta = obs_theta;
  dynamic_interval = interval;
  while (theta < end_theta) {
    theta += dynamic_interval;
    if (theta > end_theta) {
      break;
    }
    dynamic_interval = DynamicInterval(theta, interval,
      start_theta, end_theta, obs_start_theta, obs_end_theta);
    later_theta.emplace_back(theta);
  }
  former_theta.insert(former_theta.end(), later_theta.begin(), later_theta.end());

  std::vector<double> border_point1, border_point2;
  grid_x_.clear();
  grid_y_.clear();
  occupied_flag_.clear();
  std::vector<double> temp_x, temp_y;
  std::vector<bool> temp_flag;
  
  
  for (auto theta : former_theta) {
    // theta += inte;
    // dynamic_interval = DynamicInterval(theta, interval,
    //   start_theta, end_theta, obs_start_theta, obs_end_theta);
    // std::cout << inte << std::endl;
    Eigen::Vector2d pos = referenceline.spline.getPostion(theta);
    Eigen::Vector2d vec_v = referenceline.spline.getDerivative(theta);
    int stage_index = mpcc.GetStage(map, pos(0), pos(1));

    border_point1 = std::vector<double>{map.outer_point_x_[stage_index],
                                        map.outer_point_y_[stage_index]};
    border_point2 = std::vector<double>{map.outer_point_x_[stage_index + 1],
                                        map.outer_point_y_[stage_index + 1]};
    std::vector<double> outer_border_point =
        GetIntersectionPoint(pos, border_point1, border_point2, vec_v);

    border_point1 = std::vector<double>{map.inner_point_x_[stage_index],
                                        map.inner_point_y_[stage_index]};
    border_point2 = std::vector<double>{map.inner_point_x_[stage_index + 1],
                                        map.inner_point_y_[stage_index + 1]};
    std::vector<double> inner_border_point =
        GetIntersectionPoint(pos, border_point1, border_point2, vec_v);

    double x_interval = (inner_border_point[0] - outer_border_point[0]) /
                        static_cast<double>(col_size + 1);
    double y_interval = (inner_border_point[1] - outer_border_point[1]) /
                        static_cast<double>(col_size + 1);

    // std::cout << x_interval << "," << y_interval << std::endl;
    temp_x.clear();
    temp_y.clear();
    temp_flag.clear();
    for (int i = 0; i < col_size; i++) {
      double x = outer_border_point[0] + x_interval * (i + 1);
      double y = outer_border_point[1] + y_interval * (i + 1);
      // std::cout << mpcc.InQuad(obstacle_pos_, x, y);
      temp_flag.emplace_back(mpcc.InQuad(obstacle_pos_, x, y));
      temp_x.emplace_back(x);
      temp_y.emplace_back(y);
    }
    // std::cout << std::endl;
    occupied_flag_.emplace_back(temp_flag);
    grid_x_.emplace_back(temp_x);
    grid_y_.emplace_back(temp_y);
  }
  row_size = grid_x_.size();
  // for (int i = 0;i<grid_x_.size();i++){
  //   for (int j = 0;j<grid_x_[0].size();j++){
  //     std::cout << occupied_flag_[i][j];
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout <<grid_y_.size() << "," << grid_x_[0].size() <<std::endl;
}

double Obstacle::DynamicInterval(double theta, double interval,
  double start_theta, double end_theta, double obs_start_theta, double obs_end_theta) {
  // std::cout << start_theta << "," << end_theta << "," << obs_start_theta << "," << obs_end_theta << std::endl;
  if (theta >= obs_start_theta && theta <= obs_end_theta) {
    return interval;
  } else if (theta < start_theta || theta > end_theta) {
    std::cout << "Interval optimizition error!" << std::endl;
    return interval;
  } else {
    double dist = std::min(std::fabs(theta - obs_start_theta), std::fabs(theta - obs_end_theta));
    // y = -ax^2 + 1
    // 保证obs_start_theta和obs_end_theta, interval=1
    // start_theta和obs_end_theta, interval=0.2
    double max_rate = 12.0;
    double max_delta_theta = std::max(std::fabs(obs_end_theta - end_theta),
      std::fabs(obs_start_theta - start_theta));
    double a = (max_rate - 1.0) / (max_delta_theta * max_delta_theta);
    // std::cout << a << std::endl;
    double rate = a * dist * dist + 1.0;
    // std::cout << rate << "," << interval << "," << rate * interval << std::endl;
    return rate * interval;
  }
  return interval;
}

void Obstacle::DPForward(Mpcc& mpcc, const Resample& referenceline,
    Eigen::SparseMatrix<double> state) {
  // TODO
  state.coeffRef(0, 0) = (grid_x_[0][0] + grid_x_[0].back()) / 2.0;
  state.coeffRef(1, 0) = 0.0;
  state.coeffRef(2, 0) = (grid_y_[0][0] + grid_y_[0].back()) / 2.0;
  state.coeffRef(3, 0) = 1.0;
  bool get_valid_next_flag;
  bool get_valid_path_flag = false;
  for (int layer = row_size - 2; layer >= 0; layer--) {
    for (int prev_index = 0; prev_index < col_size; prev_index++) {
      if (layer > 0 && occupied_flag_[layer - 1][prev_index]) {
        continue;
      }
      for (int now_index = 0; now_index < col_size; now_index++) {
        if (occupied_flag_[layer][now_index]) {
          // 之前这里没有赋值，导致dp搜索有问题，已修复
          optimal_cost[layer + 1][now_index][prev_index] = dead_cost;
          optimal_index[layer + 1][now_index][prev_index] = -1;
          continue;
        }
        get_valid_next_flag = false;
        double min_cost;
        int min_cost_next_index;
        for (int next_index = 0; next_index < col_size; next_index++) {
          if (PathIsObstructed(layer, now_index, next_index)) {
            // std::cout << layer << "," << prev_index << "," << now_index << "," << next_index << std::endl;
            if (next_index == col_size - 1 && !get_valid_next_flag) {
              // std::cout << "......." << std::endl;
              optimal_cost[layer + 1][now_index][prev_index] = dead_cost;
              optimal_index[layer + 1][now_index][prev_index] = -1;
              break;
            }
            continue;
          }
          double similarity_cost =
              GetSimilarityCost(referenceline, layer, next_index);
          double length_cost =
              GetLengthCost(mpcc, layer, now_index, next_index, state);
          double angle_cost =
              GetAngleCost(mpcc, layer, prev_index, now_index, next_index, state);
          // std::cout<<similarity_cost<<","<<length_cost<<","<<angle_cost<<std::endl;
          double total_cost;
          if (layer == row_size - 2) {
            total_cost = similarity_weight * similarity_cost +
                         length_weight * length_cost +
                         angle_weight * angle_cost;
          } else {
            total_cost = similarity_weight * similarity_cost +
                         length_weight * length_cost +
                         angle_weight * angle_cost +
                         time_step_cost_discount_factor *
                             optimal_cost[layer + 2][next_index][now_index];
          }
          // if (layer == 11 && prev_index == 0 && now_index == 3 && next_index == 4) {
          //   std::cout << "scshjksc:" << total_cost << "," << optimal_cost[layer + 2][next_index][now_index] << std::endl;
          // }
          // if (layer == 11 && prev_index == 2 && now_index == 3 && next_index == 4) {
          //   std::cout << "bfbfbf:" << total_cost << "," << optimal_cost[layer + 2][next_index][now_index] << std::endl;
          // }
          //第一次到这
          if (!get_valid_next_flag) {
            min_cost = total_cost;
            min_cost_next_index = next_index;
            get_valid_next_flag = true;
          } else {
            if (total_cost < min_cost) {
              min_cost = total_cost;
              min_cost_next_index = next_index;
            }
          }
        }
        if (get_valid_next_flag) {
          optimal_cost[layer + 1][now_index][prev_index] = min_cost;
          optimal_index[layer + 1][now_index][prev_index] = min_cost_next_index;
          // std::cout << layer + 1 << "," << now_index << "," << prev_index <<
          // ","
          //           << min_cost_next_index << std::endl;
          get_valid_path_flag = true;
        }
      }
      if (!get_valid_next_flag) {
        std::cout << "no_valid_path : " << layer << "," << prev_index << std::endl;
      }
      // 第一层点时，前面只有无人机一个点，prev_index只作一次循环即可，在此break
      if (layer == 0) {
        break;
      }
    }
  }
  // 无人机层还要计算
  int layer = -1;
  get_valid_next_flag = false;
  double min_cost;
  int min_cost_next_index;
  for (int next_index = 0; next_index < col_size; next_index++) {
    if (occupied_flag_[layer + 1][next_index]) {
      if (next_index == col_size - 1 && !get_valid_next_flag) {
        break;
      }
      continue;
    }
    double similarity_cost =
        GetSimilarityCost(referenceline, layer, next_index);
    double length_cost = GetLengthCost(mpcc, layer, 0, next_index, state);
    double angle_cost = GetAngleCost(mpcc, layer, 0, 0, next_index, state);
    double total_cost =
        similarity_weight * similarity_cost + length_weight * length_cost +
        angle_weight * angle_cost +
        time_step_cost_discount_factor * optimal_cost[layer + 2][next_index][0];
    if (!get_valid_next_flag) {
      min_cost = total_cost;
      min_cost_next_index = next_index;
      get_valid_next_flag = true;
    } else {
      if (total_cost < min_cost) {
        min_cost = total_cost;
        min_cost_next_index = next_index;
      }
    }
  }
  optimal_cost[0][0][0] = min_cost;
  optimal_index[0][0][0] = min_cost_next_index;
}

void Obstacle::DPBackward(Mpcc& mpcc) {
  std::vector<double> optimal_path_x;
  std::vector<double> optimal_path_y;
  
  optimal_path.clear();
  optimal_path_x.clear();
  optimal_path_y.clear();
  optimal_path.resize(row_size);
  optimal_path[0] = optimal_index[0][0][0];
  optimal_path_x.emplace_back(grid_x_[0][optimal_path[0]]);
  optimal_path_y.emplace_back(grid_y_[0][optimal_path[0]]);
  optimal_path[1] = optimal_index[1][optimal_path[0]][0];
  optimal_path_x.emplace_back(grid_x_[1][optimal_path[1]]);
  optimal_path_y.emplace_back(grid_y_[1][optimal_path[1]]);
  
  for (int i = 2; i < row_size; i++) {
    optimal_path[i] =
        optimal_index[i][optimal_path[i - 1]][optimal_path[i - 2]];
    optimal_path_x.emplace_back(grid_x_[i][optimal_path[i]]);
    optimal_path_y.emplace_back(grid_y_[i][optimal_path[i]]);
  }
  mpcc.optimal_path_x = optimal_path_x;
  mpcc.optimal_path_y = optimal_path_y;
}

void Obstacle::ExpandPath(Mpcc& mpcc) {
  // 第一个代表无人机点， 先这么用
  optimal_path[0] = optimal_path[1];
  std::vector<double> left_border_x;
  std::vector<double> left_border_y;
  std::vector<double> right_border_x;
  std::vector<double> right_border_y;
  // // 左边界最内侧点
  // int left_most_inner_row = 0;
  // int left_most_inner_col = 0;
  // // 左边界第一个不为零点
  // int left_first_row = 0;
  // int left_first_col = 0;
  // // 左边界最后一个不为零点
  // int left_last_row = row_size - 1;
  // int left_last_col = 0;

  // bool left_flag = false;

  // // 右边界最内侧点
  // int right_most_inner_row = 0;
  // int right_most_inner_col = col_size - 1;
  // // 右边界第一个不为零点
  // int right_first_row = 0;
  // int right_first_col = 0;
  // // 右边界最后一个不为零点
  // int right_last_row = row_size - 1;
  // int right_last_col = 0;

  // bool right_flag = false;


  for (int i = 0; i < optimal_path.size(); i++) {
    int left = optimal_path[i];
    int right = optimal_path[i];
    // 向左
    for (int j = optimal_path[i] - 1; j >= 0; j--) {
      if (occupied_flag_[i][j]) {
        break;
      }
      left = j;
    }
    // if (left > left_most_inner_col) {
    //   left_most_inner_row = i;
    //   left_most_inner_col = left;
    // }
    // if (left != 0 && !left_flag) {
    //   left_first_row = i;
    //   left_first_col = left;
    //   left_flag = true;
    // }
    // if (left != 0) {
    //   left_last_row = i;
    //   left_last_col = left;
    // }
    double left_x, left_y;
    if (left == 0) {
      left_x = 2 * grid_x_[i][0] - grid_x_[i][1];
      left_y = 2 * grid_y_[i][0] - grid_y_[i][1];
    } else {
      double x, y;
      for (double rate = 1.0; rate > 0.0; rate -= 0.02) {
        x = grid_x_[i][left - 1] + rate * (grid_x_[i][left] - grid_x_[i][left - 1]);
        y = grid_y_[i][left - 1] + rate * (grid_y_[i][left] - grid_y_[i][left - 1]);
        if (mpcc.InQuad(obstacle_pos_, x, y)) {
          break;
        }
      }
      left_x = x;
      left_y = y;
    }
    // left_border_x.emplace_back(grid_x_[i][left]);
    // left_border_y.emplace_back(grid_y_[i][left]);
    left_border_x.emplace_back(left_x);
    left_border_y.emplace_back(left_y);

    // 向右
    for (int j = optimal_path[i] + 1; j < col_size; j++) {
      if (occupied_flag_[i][j]) {
        break;
      }
      right = j;
    }
    // // std::cout << "right:" << right << std::endl;
    // if (right < right_most_inner_col) {
    //   right_most_inner_row = i;
    //   right_most_inner_col = right;
    //   }
    // if (right != col_size - 1 && !right_flag) {
    //   right_first_row = i;
    //   right_first_col = right;
    //   right_flag = true;
    // }
    // if (right != col_size - 1) {
    //   right_last_row = i;
    //   right_last_col = right;
    // }
    double right_x, right_y;
    if (right == col_size - 1) {
      right_x = 2 * grid_x_[i][col_size - 1] - grid_x_[i][col_size - 2];
      right_y = 2 * grid_y_[i][col_size - 1] - grid_y_[i][col_size - 2];
    } else {
      double x, y;
      for (double rate = 1.0; rate > 0.0; rate -= 0.02) {
        x = grid_x_[i][right + 1] + rate * (grid_x_[i][right] - grid_x_[i][right + 1]);
        y = grid_y_[i][right + 1] + rate * (grid_y_[i][right] - grid_y_[i][right + 1]);
        if (mpcc.InQuad(obstacle_pos_, x, y)) {
          break;
        }
      }
      right_x = x;
      right_y = y;
    }

    // right_border_x.emplace_back(grid_x_[i][right]);
    // right_border_y.emplace_back(grid_y_[i][right]);
    right_border_x.emplace_back(right_x);
    right_border_y.emplace_back(right_y);

    // std::cout << i << "," << optimal_path[i] << std::endl;
  }
  // // 作插值，暂不启用
  // // 左侧有障碍物
  // if (left_most_inner_col > 0) {
  //   for (int i = 0; i < row_size; i++) {
  //     if (i <  left_first_row) {
  //       left_border_x[i] = i * (left_border_x[left_first_row] - left_border_x[0]) / left_first_row + left_border_x[0];
  //       left_border_y[i] = i * (left_border_y[left_first_row] - left_border_y[0]) / left_first_row + left_border_y[0];
  //     }
  //     if (i >  left_last_row) {
  //       left_border_x[i] = left_border_x[row_size - 1] - (row_size - 1 - i) * (left_border_x[row_size - 1] - left_border_x[left_last_row]) / (row_size - 1 - left_last_row);
  //       left_border_y[i] = left_border_y[row_size - 1] - (row_size - 1 - i) * (left_border_y[row_size - 1] - left_border_y[left_last_row]) / (row_size - 1 - left_last_row);
  //     }
  //   }
  // }
  // // 右侧有障碍物
  // if (right_most_inner_col < col_size - 1) {
  //   for (int i = 0; i < row_size; i++) {
  //     if (i <  right_first_row) {
  //       right_border_x[i] = i * (right_border_x[right_first_row] - right_border_x[0]) / right_first_row + right_border_x[0];
  //       right_border_y[i] = i * (right_border_y[right_first_row] - right_border_y[0]) / right_first_row + right_border_y[0];
  //     }
  //     if (i >  right_last_row) {
  //       right_border_x[i] = right_border_x[row_size - 1] - (row_size - 1 - i) * (right_border_x[row_size - 1] - right_border_x[right_last_row]) / (row_size - 1 - right_last_row);
  //       right_border_y[i] = right_border_y[row_size - 1] - (row_size - 1 - i) * (right_border_y[row_size - 1] - right_border_y[right_last_row]) / (row_size - 1 - right_last_row);
  //       std::cout <<  right_border_x[i] << "," << right_border_y[i] << "," << (row_size - 1 - i) / (row_size - 1 - right_last_row) << std::endl;     
  //     }
  //   }
  // }
  
  mpcc.left_border_x = left_border_x;
  mpcc.left_border_y = left_border_y;
  mpcc.right_border_x = right_border_x;
  mpcc.right_border_y = right_border_y;
  
  // std::cout << left_most_inner_row << "," << left_most_inner_col << "," << right_most_inner_row << "," << right_most_inner_col << std::endl;
  // std::cout << left_first_row << "," << left_first_col << "," << right_first_row << "," << right_first_col << std::endl;
  // std::cout << left_last_row << "," << left_last_col << "," << right_last_row << "," << right_last_col << std::endl;
}



bool Obstacle::PathIsObstructed(int layer, int now_index, int next_index) {
  if (now_index < next_index) {
    for (int i = now_index + 1; i < next_index; i++) {
      if (occupied_flag_[layer][i] || occupied_flag_[layer + 1][i]) {
        return true;
      }
    }
  } else {
    for (int i = next_index + 1; i < now_index; i++) {
      if (occupied_flag_[layer][i] || occupied_flag_[layer + 1][i]) {
        return true;
      }
    }
  }
  return false;
}

double Obstacle::GetSimilarityCost(const Resample& referenceline, int layer,
                                   int next_index) {
  double x = grid_x_[layer + 1][next_index];
  double y = grid_y_[layer + 1][next_index];
  double theta = referenceline.spline.porjectOnSpline(x, y);
  Eigen::Vector2d pos = referenceline.spline.getPostion(theta);
  return std::sqrt(std::pow(x - pos(0), 2) + std::pow(y - pos(1), 2));
}

double Obstacle::GetLengthCost(Mpcc& mpcc, int layer, int now_index,
                               int next_index, Eigen::SparseMatrix<double> state) {
  double x1, y1;
  if (layer >= 0) {
    x1 = grid_x_[layer][now_index];
    y1 = grid_y_[layer][now_index];
  } else {
    x1 = state.coeffRef(0, 0);
    y1 = state.coeffRef(2, 0);
  }

  double x2 = grid_x_[layer + 1][next_index];
  double y2 = grid_y_[layer + 1][next_index];
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

double Obstacle::GetAngleCost(Mpcc& mpcc, int layer, int prev_index,
                              int now_index, int next_index,
                              Eigen::SparseMatrix<double> state) {
  double x1, y1, x2, y2, x3, y3;
  x3 = grid_x_[layer + 1][next_index];
  y3 = grid_y_[layer + 1][next_index];
  double dx1, dy1, dx2, dy2;
  if (layer == -1) {
    dx1 = state.coeffRef(1, 0);
    dy1 = state.coeffRef(3, 0);
    dx2 = x3 - state.coeffRef(0, 0);
    dy2 = y3 - state.coeffRef(2, 0);
  } else {
    if (layer == 0) {
      x1 = state.coeffRef(0, 0);
      y1 = state.coeffRef(2, 0);
    } else {
      x1 = grid_x_[layer - 1][prev_index];
      y1 = grid_y_[layer - 1][prev_index];
    }
    x2 = grid_x_[layer][now_index];
    y2 = grid_y_[layer][now_index];
    dx1 = x2 - x1;
    dy1 = y2 - y1;
    dx2 = x3 - x2;
    dy2 = y3 - y2;
  }

  // double numer = dx1 * dx2 + dy1 * dy2;
  // double denom = std::sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
  // double ratio = numer / denom;
  // if (ratio > 1) {
  //   ratio = 1;
  // } else if (ratio < -1) {
  //   ratio = -1;
  // }
  // return -acos(ratio);

  double dx = dx1 - dx2;
  double dy = dy1 - dy2;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<double> Obstacle::GetIntersectionPoint(
    Eigen::Vector2d pos, std::vector<double> border_point1,
    std::vector<double> border_point2, Eigen::Vector2d vec_v) {
  // get vertical vector by spline and get crossover point
  // border : border_point1 , border_point2
  // intersection : (x1, y1)
  // now : now_point
  double x0 = pos(0);
  double y0 = pos(1);
  double x2 = border_point1[0];
  double y2 = border_point1[1];
  double x3 = border_point2[0];
  double y3 = border_point2[1];

  // Cramer Rule
  double a11 = y3 - y2;
  double a12 = -(x3 - x2);
  double b1 = x2 * (y3 - y2) - y2 * (x3 - x2);
  double a21 = vec_v(0);
  double a22 = vec_v(1);
  double b2 = x0 * vec_v(0) + y0 * vec_v(1);
  double D = a11 * a22 - a12 * a21;
  double D1 = b1 * a22 - b2 * a12;
  double D2 = a11 * b2 - b1 * a21;

  double x1 = D1 / D;
  double y1 = D2 / D;

  return std::vector<double>{x1, y1};
}