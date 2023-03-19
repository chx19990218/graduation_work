// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "smooth.h"

void Smooth::Fem(const Search& search) {
  int N = search.result_x.size();
  matrix_a_smooth_ = Eigen::MatrixXd::Zero(2 * N - 4, 2 * N);
  matrix_a_length_ = Eigen::MatrixXd::Zero(2 * N - 2, 2 * N);
  for (int i = 0; i < 2 * N - 4; i++) {
    matrix_a_smooth_(i, i) = 1.0;
    matrix_a_smooth_(i, i + 2) = -2.0;
    matrix_a_smooth_(i, i + 4) = 1.0;
  }
  for (int i = 0; i < 2 * N - 2; i++) {
    matrix_a_length_(i, i) = 1.0;
    matrix_a_length_(i, i + 2) = -1.0;
  }
  // double w1 = 0.05;
  // double w2 = 0.5;
  // double w3 = 1.0;
  double w1 = 0.05;
  double w2 = 0.2;
  double w3 = 1.0;
  double w3_end = 2.0 * w3;
  matrix_a_similarity_ = w3 * Eigen::MatrixXd::Identity(2 * N, 2 * N);
  matrix_a_similarity_(0, 0) = w3_end;
  matrix_a_similarity_(1, 1) = w3_end;
  matrix_a_similarity_(2 * N - 1, 2 * N - 1) = w3_end;
  matrix_a_similarity_(2 * N - 2, 2 * N - 2) = w3_end;
  auto H = 2 * (w1 * matrix_a_smooth_.transpose() * matrix_a_smooth_ +
                w2 * matrix_a_length_.transpose() * matrix_a_length_ +
                matrix_a_similarity_);
  Eigen::VectorXd Xr(2 * N);
  for (int i = 0; i < N; i++) {
    double rate = (i == 0 || i == N - 1) ? w3_end : w3;
    Xr[2 * i] = rate * search.result_x[i];
    Xr[2 * i + 1] = rate * search.result_y[i];
  }
  // J = x'Hx/2 + f'x
  // x = -H^-1 * f
  // H = 2*(w1*A1'*A1 + w2*A2'*A2 + w3*E)
  // f = -2Xr
  auto res = 2* H.inverse() * Xr;
  for (int i = 0; i < N; i++) {
    result_x.emplace_back(res[2 * i]);
    result_y.emplace_back(res[2 * i + 1]);
  }
}