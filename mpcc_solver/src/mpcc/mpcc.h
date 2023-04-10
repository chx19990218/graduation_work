// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include <quadrotor_msgs/PositionCommand.h>

#include "osqp_interface.h"
#include "resample.h"
#include "sparse_utils.h"
#include "config.h"

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
  int horizon;
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
  Eigen::MatrixXd Q_deltau;
  Eigen::MatrixXd Q_u;

  Eigen::SparseMatrix<double> inputPredict;
  Eigen::SparseMatrix<double> statePredict;
  Eigen::SparseMatrix<double> progress;

  std::vector<double> optimal_theta = std::vector<double>(horizon);

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
  std::vector<std::vector<double>> obstacle_pos_{
      {-0.2, 1.6}, {-0.2, 1.2}, {0.2, 1.2}, {0.2, 1.6}};
  // std::vector<std::vector<double>> obstacle_pos_{
  //     {-1.6, -1.2}, {-1.0, -1.2}, {-1.0, -1.0}, {-1.6, -1.0}};

  quadrotor_msgs::PositionCommand cmdMsg;

  bool mpcc_valid_flag_ = false;
  bool init_flag = true;
  bool start_test_flag = false;
  double max_cmd_a = 0.0;
  ros::Time start_test_time;

  int output_index = 0;

  Mpcc(const Config& config);
  void Init(const Resample& referenceline, Eigen::SparseMatrix<double> state,
      const Config& config);
  void UpdateState(const Resample& referenceline, Eigen::SparseMatrix<double>& state);
  void CalculateCost(const Resample& referenceline, const Config& config,
    Eigen::SparseMatrix<double> state);
  void RecedeOneHorizon(const Resample& referenceline);
  void SolveQp(const Resample& referenceline, const Map& map, const Config& config,
    Eigen::SparseMatrix<double> state);
  int GetStage(const Map& map, double x, double y);
  bool InRec(std::vector<std::vector<double>>& rec, double x, double y);
  bool InQuad(std::vector<std::vector<double>>& rec, double x, double y);
  void SetConstrains(const Resample& referenceline, const Map& map,
    Eigen::SparseMatrix<double> state, const Config& config);
  std::vector<double> GetPointBorderConstrain(const Map& map, double x,
                                              double, const Resample& referenceline, const Config& config);
  std::vector<double> GetVerticalPoint(double x1, double y1, double x2,
                                       double y2, double x3, double y3);
  void GetRefPoint(const Resample& referenceline, const double s,
                   std::vector<double>& ref_point);
  std::vector<double> GetBorder(double now_x, double now_y);
  void GetErrorInfo(const Resample& referenceline, const Stage& stage,
                    std::vector<double>& error,
                    Eigen::SparseMatrix<double>& dEc,
                    Eigen::SparseMatrix<double>& dEl);
  void chance_constrains_set(std::vector<double>& coeff, std::vector<double> obst,
      std::vector<double> ego, const Config& config);
  double normsinv(const double p);
  void UpdateResultForPlot(const Resample& referenceline,
      Eigen::SparseMatrix<double> state);
  bool InCorridorRange(const Map& map, double x, double y);
  double GetKappa(std::vector<std::vector<double>> points);
  void CircleTest(Eigen::SparseMatrix<double> state, std::vector<double>& cmd, const Config& config);
};