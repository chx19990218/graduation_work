// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include "mpcc.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "sparse_utils.h"
#include "resample.h"
#include "osqp_interface.h"



struct Stage {
  //x vx y vy z vz theta
  int state_dim_ = 7;
  //ax ay az v_theta
  int control_dim_ = 4;
  Eigen::SparseMatrix<double> Qn;
  Eigen::SparseMatrix<double> qn;
  std::vector<double> state;
  std::vector<double> u;
  Stage() {
    state = std::vector<double>(state_dim_, 0.0);;
    u = std::vector<double>(control_dim_, 0.0);
    Qn.resize(state_dim_, state_dim_);
    qn.resize(state_dim_, 1);
  };
};

class Mpcc {
 public:
  //x vx y vy z vz theta
  int state_dim_ = 7;
  //ax ay az v_theta
  int control_dim_ = 4;
  int horizon = 10;
  bool init_status = true;
  double Ts = 0.02;
  double max_theta_;
  std::vector<Stage> stage;// = std::vector<Stage>(horizon);

  OSQPInterface osqpInterface;

  Eigen::SparseMatrix<double> Q;
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
  Eigen::SparseMatrix<double> A ;
  Eigen::SparseMatrix<double> b ;

  Eigen::SparseMatrix<double> inputPredict;
  Eigen::SparseMatrix<double> statePredict;

  std::vector<double> optimal_theta = std::vector<double>(horizon);

  Mpcc();
  void CalculateCost(const Resample& referenceline, Eigen::SparseMatrix<double>& x0);
  void GetOptimalTheta();
  void SolveQp(Eigen::SparseMatrix<double>& stateTmp);
};