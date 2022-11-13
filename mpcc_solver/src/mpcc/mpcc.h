// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include "osqp_interface.h"
#include "resample.h"
#include "sparse_utils.h"

struct Stage {
  // x vx y vy theta
  int state_dim_ = 5;
  // ax ay v_theta
  int control_dim_ = 3;

  std::vector<double> state;
  std::vector<double> u;

  Stage(std::vector<double> state_) { state = state_; }
};

class Mpcc {
 public:
  // x vx y vy z vz theta
  int state_dim_ = 5;
  // ax ay az v_theta
  int control_dim_ = 3;
  int horizon = 20;
  bool init_status = true;
  double Ts = 0.02;
  double max_theta_;
  std::vector<Stage> stage;  // = std::vector<Stage>(horizon);

  OSQPInterface osqpInterface;

  Eigen::SparseMatrix<double> Q;
  Eigen::SparseMatrix<double> identity;
  Eigen::SparseMatrix<double> q;
  Eigen::SparseMatrix<double> Ad;
  Eigen::SparseMatrix<double> Bd;
  Eigen::SparseMatrix<double> AA;
  Eigen::SparseMatrix<double> BB;
  Eigen::SparseMatrix<double> AAT;
  Eigen::SparseMatrix<double> BBT;
  Eigen::SparseMatrix<double> H;
  Eigen::SparseMatrix<double> f;
  Eigen::SparseMatrix<double> C;
  Eigen::SparseMatrix<double> xu;
  Eigen::SparseMatrix<double> xl;
  Eigen::SparseMatrix<double> uu;
  Eigen::SparseMatrix<double> ul;
  Eigen::SparseMatrix<double> clow;
  Eigen::SparseMatrix<double> cupp;
  Eigen::SparseMatrix<double> A;
  Eigen::SparseMatrix<double> b;
  Eigen::SparseMatrix<double> Cx;
  Eigen::SparseMatrix<double> xup;
  Eigen::SparseMatrix<double> xlow;

  Eigen::SparseMatrix<double> inputPredict;
  Eigen::SparseMatrix<double> statePredict;

  std::vector<double> optimal_theta = std::vector<double>(horizon);
  Eigen::SparseMatrix<double> state;
  std::vector<double> x_history, y_history;
  std::vector<double> x_horizon, y_horizon;
  std::vector<double> theta_x_;
  std::vector<double> theta_y_;

  std::vector<double> optimal_path_x;
  std::vector<double> optimal_path_y;
  std::vector<double> left_border_x;
  std::vector<double> left_border_y;
  std::vector<double> right_border_x;
  std::vector<double> right_border_y;

  Mpcc();
  void Init(const Resample& referenceline);
  void UpdateState(const Resample& referenceline);
  void CalculateCost(const Resample& referenceline);
  void RecedeOneHorizon(const Resample& referenceline);
  void SolveQp(const Resample& referenceline, const Map& map);
  int GetStage(const Map& map, double x, double y);
  bool InRec(std::vector<std::vector<double>>& rec, double x, double y);
  void SetConstrains(const Resample& referenceline, const Map& map);
  std::vector<double> GetPointBorderConstrain(const Map& map, double x,
                                              double y);
  std::vector<double> GetVerticalPoint(double x1, double y1, double x2,
                                       double y2, double x3, double y3);
  void GetRefPoint(const Resample& referenceline, const double s,
                   std::vector<double>& ref_point);
  std::vector<double> GetBorder(double now_x, double now_y);
  void GetErrorInfo(const Resample& referenceline, const Stage& stage,
                    std::vector<double>& error,
                    Eigen::SparseMatrix<double>& dEc,
                    Eigen::SparseMatrix<double>& dEl);
};