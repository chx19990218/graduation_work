// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "mpcc.h"

Mpcc::Mpcc() {
  Q.resize(horizon * state_dim_, horizon * state_dim_);
  identity.resize(horizon * state_dim_, horizon * state_dim_);
  for (int i = 0; i < horizon * state_dim_; i++) {
    identity.coeffRef(i, i) = 1e-12;
  }
  q.resize(horizon * state_dim_, 1);
  AA.resize(horizon * state_dim_, state_dim_);
  BB.resize(horizon * state_dim_, horizon * control_dim_);

  Ad.resize(state_dim_, state_dim_);
  std::vector<Eigen::Triplet<double>> triplets;
  for (int i = 0; i < state_dim_; i++) {
    triplets.push_back(Eigen::Triplet<double>(i, i, 1));
  }
  triplets.push_back(Eigen::Triplet<double>(0, 1, Ts));
  triplets.push_back(Eigen::Triplet<double>(2, 3, Ts));
  Ad.setFromTriplets(triplets.begin(), triplets.end());

  Bd.resize(state_dim_, control_dim_);
  triplets.clear();
  triplets.push_back(Eigen::Triplet<double>(1, 0, Ts));
  triplets.push_back(Eigen::Triplet<double>(3, 1, Ts));
  triplets.push_back(Eigen::Triplet<double>(4, 2, Ts));
  Bd.setFromTriplets(triplets.begin(), triplets.end());

  // initialize AA & BB:
  Eigen::SparseMatrix<double> tmpA = Ad;
  Eigen::SparseMatrix<double> tmpB(state_dim_, horizon * control_dim_);
  sp::colMajor::setRows(AA, Ad, 0);
  sp::colMajor::setBlock(BB, Bd, 0, 0);
  for (int k = 1; k < horizon; k++) {
    tmpA = Ad * tmpA;
    sp::colMajor::setRows(AA, tmpA, k * state_dim_);
    tmpB = Ad * BB.block((k - 1) * state_dim_, 0, state_dim_,
                         horizon * control_dim_);
    // 先把上一行乘Ad挪下来
    sp::colMajor::setRows(BB, tmpB, k * state_dim_);
    // 再加上新的
    sp::colMajor::setBlock(BB, Bd, k * state_dim_, k * control_dim_);
  }
  AAT = AA.transpose();
  BBT = BB.transpose();
  inputPredict.resize(control_dim_, horizon);
  statePredict.resize(state_dim_, horizon);
}

void Mpcc::Init(const Resample& referenceline, Eigen::SparseMatrix<double> state) {
  max_theta_ = referenceline.spline.getLength(); 
  optimal_theta.clear();
  // 初速度为0,初始轨迹为匀加速
  double v = 1.0;
  double a = v / (Ts * horizon);
  double x, y, theta;
  for (int i = 0; i < horizon; i++) {
    theta = state.coeffRef(state_dim_ - 1, 0) + a * i * i * Ts * Ts / 2;
    auto pos_xy = referenceline.spline.getPostion(theta);
    auto dpos_xy = referenceline.spline.getDerivative(theta);
    double phi = atan2(dpos_xy(1), dpos_xy(0));

    std::vector<double> new_state{pos_xy[0], std::cos(phi) * a * i * Ts,
                                  pos_xy[1], std::sin(phi) * a * i * Ts, theta};
    Stage stage_i(new_state);
    stage.emplace_back(stage_i);
    optimal_theta.emplace_back(theta);
  }
}

// 投影更新theta
void Mpcc::UpdateState(const Resample& referenceline,
    Eigen::SparseMatrix<double>& state) {
  double now_x = state.coeffRef(0, 0);
  double now_y = state.coeffRef(2, 0);
  double now_theta = referenceline.spline.porjectOnSpline(now_x, now_y);
  state.coeffRef(state_dim_ - 1, 0) = now_theta;
}

void Mpcc::RecedeOneHorizon(const Resample& referenceline) {
  optimal_theta.emplace_back(std::fmod(
      2 * optimal_theta.back() - optimal_theta.rbegin()[1], max_theta_));
  optimal_theta.erase(optimal_theta.begin());

  double x = 2 * stage.back().state[0] - stage.rbegin()[1].state[0];
  double y = 2 * stage.back().state[2] - stage.rbegin()[1].state[2];

  std::vector<double> new_state = stage.back().state;
  new_state[0] = x;
  new_state[2] = y;
  new_state[4] = optimal_theta.back();
  Stage new_stage(new_state);
  stage.erase(stage.begin());
  stage.emplace_back(new_stage);
}

void Mpcc::CalculateCost(const Resample& referenceline,
    Eigen::SparseMatrix<double> state) {
  double Qc = 1.0;
  double Ql = 5.0;
  double gamma = 10.0;
  // double gamma = 0.001;
  Eigen::SparseMatrix<double> Qn;
  Eigen::SparseMatrix<double> qn;
  for (int i = 0; i < horizon; i++) {
    if (false) {
      double theta = optimal_theta[i];  // TODO 考虑闭环，theta跑一圈
      auto pos_xy = referenceline.spline.getPostion(theta);
      auto vel_xy = referenceline.spline.getDerivative(theta);
      double x = pos_xy[0];
      double y = pos_xy[1];

      double dx_dtheta = vel_xy[0];
      double dy_dtheta = vel_xy[1];

      double r_x = x - dx_dtheta * theta;
      double r_y = y - dy_dtheta * theta;

      Eigen::SparseMatrix<double> grad_x(state_dim_, 1);
      Eigen::SparseMatrix<double> grad_y(state_dim_, 1);

      grad_x.coeffRef(0, 0) = 1.0;
      grad_y.coeffRef(2, 0) = 1.0;

      grad_x.coeffRef(state_dim_ - 1, 0) = -dx_dtheta;
      grad_y.coeffRef(state_dim_ - 1, 0) = -dy_dtheta;

      Qn = 2 * grad_x * Eigen::SparseMatrix<double>(grad_x.transpose()) +
           2 * grad_y * Eigen::SparseMatrix<double>(grad_y.transpose());
      qn = -2 * r_x * grad_x - 2 * r_y * grad_y;
    } else {
      Eigen::SparseMatrix<double> X(state_dim_, 1);
      for (int j = 0; j < state_dim_; j++) {
        X.coeffRef(j, 0) = stage[i].state[j];
      }
      
      std::vector<double> error(2, 0.0);
      Eigen::SparseMatrix<double> dEc(1, state_dim_);
      Eigen::SparseMatrix<double> dEl(1, state_dim_);
      GetErrorInfo(referenceline, stage[i], error, dEc, dEl);
      double gain = 1.0;
      if (i == horizon - 1) {
        gain = 1.0;
      }
      Qn = 2 * Eigen::SparseMatrix<double>(dEc.transpose()) * gain * Qc * dEc +
           2 * Eigen::SparseMatrix<double>(dEl.transpose()) * gain * Ql * dEl;
      Eigen::SparseMatrix<double> c = dEc * X;
      Eigen::SparseMatrix<double> l = dEl * X;
      qn = 2 * gain * Qc * (error[0] - c.coeffRef(0, 0)) *
               Eigen::SparseMatrix<double>(dEc.transpose()) +
           2 * gain * Ql * (error[1] - l.coeffRef(0, 0)) *
               Eigen::SparseMatrix<double>(dEl.transpose());
    }
    bool chance_constrain_flag = true;
    double Sobs = 0.01;
    double x0 = 0.5;
    double y0 = 3.0;
    
    if (chance_constrain_flag) {
      std::vector<double> obst{x0, y0};
      std::vector<double> ego{stage[i].state[0], stage[i].state[2]};
      std::vector<double> coeff(6, 0.0);
      chance_constrains_set(coeff, obst, ego);

      double g11 = coeff[0];
      double g12 = coeff[1];
      double g21 = coeff[2];
      double g22 = coeff[3];
      double gx = coeff[4];
      double gy = coeff[5];
      
      double p11 = gx - stage[i].state[0] * g11 - stage[i].state[2] * g21;
      double p12 = gy - stage[i].state[0] * g12 - stage[i].state[2] * g22;

      Qn.coeffRef(0, 0) += -Sobs * g11;
      Qn.coeffRef(0, 2) += -Sobs * g12;
      Qn.coeffRef(2, 0) += -Sobs * g21;
      Qn.coeffRef(2, 2) += -Sobs * g22;

      qn.coeffRef(0, 0) += -Sobs * p11;
      qn.coeffRef(2, 0) += -Sobs * p12;
    }
    sp::colMajor::setBlock(Q, Qn, state_dim_ * i, state_dim_ * i);
    sp::colMajor::setBlock(q, qn, state_dim_ * i, 0);
  }

  Eigen::SparseMatrix<double> progress(control_dim_ * horizon, 1);
  for (int i = 0; i < horizon; i++) {
    progress.coeffRef(control_dim_ * i + control_dim_ - 1, 0) = gamma;
  }
  H = BBT * Q * BB;
  f = BBT * Q * AA * state + BBT * q - progress;
}

void Mpcc::GetRefPoint(const Resample& referenceline, const double s,
                       std::vector<double>& ref_point) {
  // X-Y postion of the reference at s
  Eigen::Vector2d pos_ref = referenceline.spline.getPostion(s);
  double x_ref = pos_ref(0);
  double y_ref = pos_ref(1);
  ref_point[0] = x_ref;
  ref_point[1] = y_ref;
  // reference path derivatives
  Eigen::Vector2d dpos_ref = referenceline.spline.getDerivative(s);
  double dx_ref = dpos_ref(0);
  double dy_ref = dpos_ref(1);
  ref_point[2] = dx_ref;
  ref_point[3] = dy_ref;
  // angle of the reference path
  double phi_ref = atan2(dy_ref, dx_ref);
  ref_point[4] = phi_ref;
  // second order derivatives
  Eigen::Vector2d ddpos_ref = referenceline.spline.getSecondDerivative(s);
  double ddx_ref = ddpos_ref(0);
  double ddy_ref = ddpos_ref(1);
  // curvature
  double dphi_ref_nom = (dx_ref * ddy_ref - dy_ref * ddx_ref);
  double dphi_ref_denom = (1 + dy_ref * dy_ref / dx_ref * dx_ref);
  if (std::fabs(dphi_ref_nom) < 1e-7) {
    dphi_ref_nom = 0;
  }
  if (std::fabs(dphi_ref_denom) < 1e-7) {
    dphi_ref_denom = 1e-7;
  }
  double dphi_ref = dphi_ref_nom / dphi_ref_denom;
  ref_point[5] = dphi_ref;
}

void Mpcc::GetErrorInfo(const Resample& referenceline, const Stage& stage,
                        std::vector<double>& error,
                        Eigen::SparseMatrix<double>& dEc,
                        Eigen::SparseMatrix<double>& dEl) {
  // compute error between reference and X-Y position of the car
  double X = stage.state[0];
  double Y = stage.state[2];
  // X_ref Y_ref dX_ref/dtheta dY_ref/dtheta theta
  std::vector<double> track_point(6, 0.0);
  GetRefPoint(referenceline, stage.state.back(), track_point);

  // contouring  error
  double contouring_error = -std::sin(track_point[4]) * (X - track_point[0]) +
                            std::cos(track_point[4]) * (Y - track_point[1]);
  // lag error
  double lag_error = std::cos(track_point[4]) * (X - track_point[0]) +
                     std::sin(track_point[4]) * (Y - track_point[1]);

  error[0] = contouring_error;
  error[1] = lag_error;
  // partial derivatives of the lag and contouring error with respect to s
  double contouring_error__theta =
      -track_point[5] * std::cos(track_point[4]) * (X - track_point[0]) -
      track_point[5] * std::sin(track_point[4]) * (Y - track_point[1]) +
      track_point[2] * std::sin(track_point[4]) -
      track_point[3] * std::cos(track_point[4]);
  // std::cout<<"contouring_error__theta:"<<std::sin(track_point[4])<<","<<std::cos(track_point[4])<<std::endl;
  double lag_error__theta =
      -track_point[5] * std::sin(track_point[4]) * (X - track_point[0]) +
      track_point[5] * std::cos(track_point[4]) * (Y - track_point[1]) -
      track_point[2] * std::cos(track_point[4]) -
      track_point[3] * std::sin(track_point[4]);

  // compute all remaining partial derivatives and store the in dError
  dEc.coeffRef(0, 0) = -std::sin(track_point[4]);
  dEc.coeffRef(0, 2) = std::cos(track_point[4]);
  dEc.coeffRef(0, state_dim_ - 1) = contouring_error__theta;

  dEl.coeffRef(0, 0) = std::cos(track_point[4]);
  dEl.coeffRef(0, 2) = std::sin(track_point[4]);
  dEl.coeffRef(0, state_dim_ - 1) = lag_error__theta;
}

void Mpcc::SolveQp(const Resample& referenceline, const Map& map,
    Eigen::SparseMatrix<double> state) {
  RecedeOneHorizon(referenceline);

  SetConstrains(referenceline, map, state);
  CalculateCost(referenceline, state);


  osqpInterface.updateMatrices(H, f, A, b, C, clow, cupp, ul, uu);
  osqpInterface.solveQP();
  auto solveStatus = osqpInterface.solveStatus();

  auto solPtr = osqpInterface.solPtr();

  if (solveStatus == OSQP_SOLVED) {
    optimal_theta.clear();
    Eigen::SparseMatrix<double> state_horizon = state;
    for (int i = 0; i < horizon; i++) {
      for (int j = 0; j < control_dim_; j++) {
        inputPredict.coeffRef(j, i) = solPtr->x[i * control_dim_ + j];
      }
      // 计算对应优化状态
      state_horizon = Ad * state_horizon + Bd * inputPredict.col(i);
      sp::colMajor::setCols(statePredict, state_horizon, i);

      // 更新最优轨迹
      optimal_theta.emplace_back(state_horizon.coeffRef(state_dim_ - 1, 0));
      for (int j = 0; j < state_dim_; j++) {
        stage[i].state[j] = state_horizon.coeffRef(j, 0);
      }
    }
  } else {
    std::cout << "no solution" << std::endl;
  }
}

void Mpcc::UpdateResultForPlot(const Resample& referenceline,
    Eigen::SparseMatrix<double> state) {
  theta_x_.clear();
  theta_y_.clear();
  x_horizon.clear();
  y_horizon.clear();
  // horizon theta
  for (int i = 0; i < horizon; i++) {
    // horizon theta
    auto pos_theta = referenceline.spline.getPostion(
      statePredict.coeffRef(state_dim_ - 1, i));
    theta_x_.emplace_back(pos_theta(0));
    theta_y_.emplace_back(pos_theta(1));

    // horizon
    x_horizon.emplace_back(statePredict.coeffRef(0, i));
    y_horizon.emplace_back(statePredict.coeffRef(2, i));
  }
  // history
  x_history.emplace_back(state.coeffRef(0, 0));
  y_history.emplace_back(state.coeffRef(2, 0));
}