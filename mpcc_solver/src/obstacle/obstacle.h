// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <math.h>

#include <vector>

#include "mpcc.h"
#include "resample.h"
#include "config.h"

class Obstacle {
 public:
  int row_size = 40;
  int col_size = 3;
  double dead_cost = 500.0;
  double similarity_weight = 1.0;
  double length_weight = 1.0;
  double angle_weight = 1.0;
  double time_step_cost_discount_factor = 1.0;
  std::vector<std::vector<double>> grid_x_, grid_y_;
  std::vector<std::vector<bool>> occupied_flag_;
  std::vector<int> optimal_path = std::vector<int> (20, col_size / 2);
  std::vector<std::vector<double>> obstacle_pos_;
  std::vector<std::vector<std::vector<double>>> optimal_cost =
      std::vector<std::vector<std::vector<double>>>(
          row_size, std::vector<std::vector<double>>(
                        col_size, std::vector<double>(col_size, 0.0)));
  std::vector<std::vector<std::vector<int>>> optimal_index =
      std::vector<std::vector<std::vector<int>>>(
          row_size, std::vector<std::vector<int>>(
                        col_size, std::vector<int>(col_size, 0)));
  
  void GenerateGridCoordinate(const Resample& referenceline, const Map& map,
                              Mpcc& mpcc, const Config& config);
  std::vector<double> GetIntersectionPoint(Eigen::Vector2d pos,
                                           std::vector<double> border_point1,
                                           std::vector<double> border_point2,
                                           Eigen::Vector2d vec_v);
  double GetSimilarityCost(const Resample& referenceline, int layer,
                           int next_index);
  double GetLengthCost(Mpcc& mpcc, int layer, int now_index, int next_index,
                      Eigen::SparseMatrix<double> state);
  double GetAngleCost(Mpcc& mpcc, int layer, int prev_index, int now_index,
                      int next_index, Eigen::SparseMatrix<double> state);
  bool PathIsObstructed(int layer, int now_index, int next_index);
  void DPForward(Mpcc& mpcc, const Resample& referenceline, Eigen::SparseMatrix<double> state);
  void DPBackward(Mpcc& mpcc);
  void ExpandPath(Mpcc& mpcc);
  
  void Update(const Resample& referenceline, const Map& map,
                      Mpcc& mpcc, Eigen::SparseMatrix<double> state, const Config& config);
  double DynamicInterval(double theta, double interval,
    double start_theta, double end_theta, double obs_start_theta, double obs_end_theta);
};