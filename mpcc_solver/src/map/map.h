// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <math.h>

#include <eigen3/Eigen/Dense>

#include "arc_length_spline.h"

#define PI acos(-1)

enum CornerType { Inner = 1, Outer = 2 };

enum VecType {
  XPos = 1,
  XNeg = 2,
  YPos = 3,
  YNeg = 4,
  FirstPhase = 5,
  SecondPhase = 6,
  ThirdPhase = 7,
  FourthPhase = 8,
  Error = 9
};

class Map {
 public:
  std::vector<std::vector<double>> center_point_{
      {0.5, 0.5}, {0.5, 4.5}, {1.5, 4.5}, {1.5, 6.5},
      {3.5, 6.5}, {3.5, 9.0}, {4.7, 9.0}, {4.7, 3.0},
      {3.0, 3.0}, {3.0, 0.5}};

  Eigen::VectorXd center_point_x_, center_point_y_;
  Eigen::VectorXd outer_point_x_, outer_point_y_;
  Eigen::VectorXd inner_point_x_, inner_point_y_;
  std::vector<std::vector<std::vector<double>>> stage;

  double width = 1.0;

  Map() {}

  void GenerateMap();
  CornerType GetCornerType(std::vector<double>& last_point,
                           std::vector<double>& now_point,
                           std::vector<double>& next_point);
  VecType GetVecType(Eigen::Vector2d& vec);
  void CalculateStageRange();
  void GenRec(std::vector<std::vector<double>>& vec,
              const std::vector<double>& now_outer_point,
              const std::vector<double>& now_inner_point,
              const std::vector<double>& next_outer_point,
              const std::vector<double>& next_inner_point);
};