// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "obstacle.h"

void Obstacle::Update(const Resample& referenceline, const Map& map,
                      Mpcc& mpcc) {
  GenerateGridCoordinate(referenceline, map, mpcc);
  DPForward(mpcc, referenceline);
  DPBackward(mpcc);
  ExpandPath(mpcc);
}

void Obstacle::GenerateGridCoordinate(const Resample& referenceline,
                                      const Map& map, Mpcc& mpcc) {
  double start_x = 0.5;
  double start_y = 2.0;
  double end_x = 0.5;
  double end_y = 4.0;

  double start_theta = referenceline.spline.porjectOnSpline(start_x, start_y);

  std::vector<double> border_point1, border_point2;
  grid_x_.clear();
  grid_y_.clear();
  occupied_flag_.clear();
  for (int i = 0; i < row_size; i++) {
    double theta = start_theta + 0.1 * i;
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

    double x_interval = (outer_border_point[0] - inner_border_point[0]) /
                        static_cast<double>(col_size + 1);
    double y_interval = (outer_border_point[1] - inner_border_point[1]) /
                        static_cast<double>(col_size + 1);
    std::vector<double> temp_x, temp_y;
    std::vector<bool> temp_flag;
    for (int i = 0; i < col_size; i++) {
      double x = inner_border_point[0] + x_interval * (i + 1);
      double y = inner_border_point[1] + y_interval * (i + 1);

      temp_flag.emplace_back(mpcc.InRec(obstacle_pos_, x, y));
      temp_x.emplace_back(x);
      temp_y.emplace_back(y);
    }
    occupied_flag_.emplace_back(temp_flag);
    grid_x_.emplace_back(temp_x);
    grid_y_.emplace_back(temp_y);
  }
}

void Obstacle::DPForward(Mpcc& mpcc, const Resample& referenceline) {
  bool get_valid_next_flag;
  bool get_valid_path_flag = false;
  for (int layer = row_size - 2; layer >= 0; layer--) {
    for (int prev_index = 0; prev_index < col_size; prev_index++) {
      if (layer > 0 && occupied_flag_[layer - 1][prev_index]) {
        continue;
      }
      for (int now_index = 0; now_index < col_size; now_index++) {
        if (occupied_flag_[layer][now_index]) {
          continue;
        }
        get_valid_next_flag = false;
        double min_cost;
        int min_cost_next_index;
        for (int next_index = 0; next_index < col_size; next_index++) {
          if (PathIsObstructed(layer, now_index, next_index)) {
            if (next_index == col_size - 1 && !get_valid_next_flag) {
              optimal_cost[layer + 1][now_index][prev_index] = dead_cost;
              optimal_index[layer + 1][now_index][prev_index] = -1;
              break;
            }
            continue;
          }
          double similarity_cost =
              GetSimilarityCost(referenceline, layer, next_index);
          double length_cost =
              GetLengthCost(mpcc, layer, now_index, next_index);
          double angle_cost =
              GetAngleCost(mpcc, layer, prev_index, now_index, next_index);
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
        std::cout << "no_valid_path!" << std::endl;
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
    double length_cost = GetLengthCost(mpcc, layer, 0, next_index);
    double angle_cost = GetAngleCost(mpcc, layer, 0, 0, next_index);
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
  std::vector<double> left_border_x;
  std::vector<double> left_border_y;
  std::vector<double> right_border_x;
  std::vector<double> right_border_y;
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
    left_border_x.emplace_back(grid_x_[i][left]);
    left_border_y.emplace_back(grid_y_[i][left]);
    // 向右
    for (int j = optimal_path[i] + 1; j < col_size; j++) {
      if (occupied_flag_[i][j]) {
        break;
      }
      right = j;
    }
    right_border_x.emplace_back(grid_x_[i][right]);
    right_border_y.emplace_back(grid_y_[i][right]);
  }
  mpcc.left_border_x = left_border_x;
  mpcc.left_border_y = left_border_y;
  mpcc.right_border_x = right_border_x;
  mpcc.right_border_y = right_border_y;
}



bool Obstacle::PathIsObstructed(int layer, int now_index, int next_index) {
  if (now_index < next_index) {
    for (int i = now_index; i <= next_index; i++) {
      if (occupied_flag_[layer][i] || occupied_flag_[layer + 1][i]) {
        return true;
      }
    }
  } else {
    for (int i = next_index; i <= now_index; i++) {
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
                               int next_index) {
  double x1, y1;
  if (layer >= 0) {
    x1 = grid_x_[layer][now_index];
    y1 = grid_y_[layer][now_index];
  } else {
    x1 = mpcc.state.coeffRef(0, 0);
    y1 = mpcc.state.coeffRef(2, 0);
  }

  double x2 = grid_x_[layer + 1][next_index];
  double y2 = grid_y_[layer + 1][next_index];
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

double Obstacle::GetAngleCost(Mpcc& mpcc, int layer, int prev_index,
                              int now_index, int next_index) {
  double x1, y1, x2, y2, x3, y3;
  x3 = grid_x_[layer + 1][next_index];
  y3 = grid_y_[layer + 1][next_index];
  double dx1, dy1, dx2, dy2;
  if (layer == -1) {
    dx1 = mpcc.state.coeffRef(1, 0);
    dy1 = mpcc.state.coeffRef(3, 0);
    dx2 = x3 - mpcc.state.coeffRef(0, 0);
    dy2 = y3 - mpcc.state.coeffRef(2, 0);
  } else {
    if (layer == 0) {
      x1 = mpcc.state.coeffRef(0, 0);
      y1 = mpcc.state.coeffRef(2, 0);
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

  double numer = dx1 * dx2 + dy1 * dy2;
  double denom = std::sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
  double ratio = numer / denom;
  if (ratio > 1) {
    ratio = 1;
  } else if (ratio < -1) {
    ratio = -1;
  }
  return acos(ratio);
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