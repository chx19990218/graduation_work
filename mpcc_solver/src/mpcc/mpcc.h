// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include "mpcc.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "sparse_utils.h"
#include "resample.h"

//x vx y vy z vz theta
int state_dim_ = 7;
//ax ay az v_theta
int control_dim_ = 4;

struct Stage {
  Eigen::SparseMatrix<double> Qn;
  Eigen::SparseMatrix<double> qn;
  std::vector<double> state = std::vector<double>(state_dim_);
  std::vector<double> u = std::vector<double>(control_dim_);
  Stage() {
    Qn.resize(state_dim_, state_dim_);
    qn.resize(state_dim_, 1);
  };
};

class Mpcc {
 public:
  int horizon = 10;
  bool init_status = true;
  double Ts = 0.02;
  std::vector<Stage> stage = std::vector<Stage>(horizon);

  Eigen::SparseMatrix<double> Q;
  Eigen::SparseMatrix<double> q;
  Eigen::SparseMatrix<double> Ad;
  Eigen::SparseMatrix<double> Bd;
  Eigen::SparseMatrix<double> AA;
  Eigen::SparseMatrix<double> BB;
  Eigen::SparseMatrix<double> AAT;
  Eigen::SparseMatrix<double> BBT;
  Eigen::SparseMatrix<double> Q_qp;
  Eigen::SparseMatrix<double> c_qp;

  Mpcc();
  void CalculateCost(const Resample& referenceline);
  void GetOptimalTheta(std::vector<double>& optimal_theta);
  void SetMpccParam(Eigen::SparseMatrix<double>& x0);
};