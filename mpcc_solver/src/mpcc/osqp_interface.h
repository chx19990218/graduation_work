// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <osqp/osqp.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

// using namespace Eigen;

class OSQPInterface {
 public:
  int exitflag = 0;

  // Workspace structures
  OSQPWorkspace *work;
  OSQPSettings *settings;
  OSQPData *data;


  OSQPInterface();
  ~OSQPInterface();
  int updateMatrices(
      Eigen::SparseMatrix<double> &Q_, Eigen::SparseMatrix<double> &c_,
      Eigen::SparseMatrix<double> &A_, Eigen::SparseMatrix<double> &b_,
      Eigen::SparseMatrix<double> &C_, Eigen::SparseMatrix<double> &clow_,
      Eigen::SparseMatrix<double> &cupp_, Eigen::SparseMatrix<double> &xlow_,
      Eigen::SparseMatrix<double> &xupp_, int warmStart = WARM_START);
  int solveQP();
  int solveStatus();
  OSQPSolution *solPtr();
};
