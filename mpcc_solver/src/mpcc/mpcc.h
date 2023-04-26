// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Theta.h>

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

struct Leader {
  double x;
  double y;
  double theta = 1.0;
  double phi;
  std::vector<std::vector<double>> vec{{-0.2, 0.2, -0.2, -0.2}, // 三角
                                        {0.0, 0.2, 0.0, -0.2}, // 横一
                                        {0.2, 0.0, -0.2, 0.0}}; // 竖一
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

  nav_msgs::Path leader_path;
  nav_msgs::Path obs_path;
  nav_msgs::Odometry obs_odom;
  
  Leader leader;

  quadrotor_msgs::PositionCommand cmdMsg;

  bool mpcc_valid_flag_ = false;
  bool init_flag = true;
  bool start_test_flag = false;
  bool path_overlap_flag = false; // 与其他有轨迹交互
  bool cluster_flag = false;
  double max_cmd_a = 0.0;
  double max_v = 0.0;
  ros::Time start_test_time;
  ros::Time start_clutter_time;
  ros::Time start_time;
  int clutter_index = 0;

  int output_index = 0;
  double uav_size = 0.1;
  double buffer = 0.02;

  Mpcc(const Config& config);
  void Init(const Resample& referenceline, Eigen::SparseMatrix<double> state,
      const Config& config, nav_msgs::Path& ego_path);
  void UpdateState(const Resample& referenceline, Eigen::SparseMatrix<double>& state);
  void CalculateCost(const Resample& referenceline, const Config& config,
    Eigen::SparseMatrix<double> state, const nav_msgs::Path& ego_path, const Map& map,
    quadrotor_msgs::Theta& theta_crowded_msg, ros::Publisher& theta_crowded_pub);
  void RecedeOneHorizon(const Resample& referenceline);
  void SolveQp(const Resample& referenceline, const Map& map,Config& config,
    Eigen::SparseMatrix<double> state, nav_msgs::Path& ego_path, const nav_msgs::Path& obs1_path,
    const nav_msgs::Odometry obs1_odom, const nav_msgs::Path& obs2_path, const nav_msgs::Odometry obs2_odom,
    quadrotor_msgs::Theta& theta_crowded_msg, ros::Publisher& theta_crowded_pub);
  int GetStage(const Map& map, double x, double y);
  bool InRec(std::vector<std::vector<double>>& rec, double x, double y);
  bool InQuad(std::vector<std::vector<double>>& rec, double x, double y);
  bool RectOverlap(std::vector<std::vector<double>>& rec1, std::vector<std::vector<double>>& rec2);
  void SetConstrains(const Resample& referenceline, const Map& map,
    Eigen::SparseMatrix<double> state, const Config& config,
    const nav_msgs::Path& ego_path, const nav_msgs::Path& obs_path);
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
  int CheckCollision(const nav_msgs::Path& ego_path, const nav_msgs::Path& obs_path);
  bool CalcuDMPC(const nav_msgs::Path& ego_path, const nav_msgs::Path& obs_path,
                     Eigen::SparseMatrix<double> state,
                     Eigen::SparseMatrix<double>& Cu, Eigen::SparseMatrix<double>& ulow,
                     Eigen::SparseMatrix<double>& uup);
  void MutiCollisionCheck(const Resample& referenceline, Config& config,
    Eigen::SparseMatrix<double> state, nav_msgs::Path& ego_path, const nav_msgs::Path& obs1_path,
    const nav_msgs::Odometry obs1_odom, const nav_msgs::Path& obs2_path,
    const nav_msgs::Odometry obs2_odom);
  bool ClusterFormation(const Resample& referenceline, Config& config,
    Eigen::SparseMatrix<double> state, const nav_msgs::Path& ego_path,
    const nav_msgs::Path& obs1_path, const nav_msgs::Odometry obs1_odom,
    const nav_msgs::Path& obs2_path, const nav_msgs::Odometry obs2_odom);
};