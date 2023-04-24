// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "mpcc.h"

Mpcc::Mpcc(const Config& config) {
  horizon = config.horizon;
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

  Q_deltau = 2.0 * Eigen::MatrixXd::Identity(control_dim_ * horizon, control_dim_ * horizon);
  Q_deltau.coeffRef(0, 0) = 1.0;
  Q_deltau.coeffRef(1, 1) = 1.0;
  Q_deltau.coeffRef(2, 2) = 1.0;
  Q_deltau.coeffRef(control_dim_ * horizon - 3, control_dim_ * horizon - 3) = 1.0;
  Q_deltau.coeffRef(control_dim_ * horizon - 2, control_dim_ * horizon - 2) = 1.0;
  Q_deltau.coeffRef(control_dim_ * horizon - 1, control_dim_ * horizon - 1) = 1.0;
  for (int i = 0; i < control_dim_ * horizon - 3; i++) {
    // H要对称正定
    Q_deltau.coeffRef(i, i + 3) = -1.0;
    Q_deltau.coeffRef(i + 3, i) = -1.0;
  }
  Q_u = Eigen::MatrixXd::Identity(control_dim_ * horizon, control_dim_ * horizon);
  for (int i = 0; i < horizon; i++) {
    Q_u.coeffRef(i * control_dim_ + 2, i * control_dim_ + 2) = 0.0;
  }
  progress.resize(control_dim_ * horizon, 1);
  for (int i = 0; i < horizon; i++) {
    progress.coeffRef(control_dim_ * i + control_dim_ - 1, 0) = Ts;
  }
  // add dmpc
  C.resize(4 * horizon + 1, BB.cols());
  cupp.resize(4 * horizon + 1, 1);
  clow.resize(4 * horizon + 1, 1);
  // 加速度/角度,里程速度限制限制
  double max_attitude = config.angle_upper_limit * PI / 180.0;
  double max_a = 9.8 * std::tan(max_attitude);
  double max_v = config.theta_dot_upper_limit;
  for (int i = 0; i < horizon; i++) {
    // x轴控制量上限
    C.coeffRef(horizon + i, i * control_dim_ + 0) = 1.0;
    cupp.coeffRef(horizon + i, 0) = max_a;
    clow.coeffRef(horizon + i, 0) = -max_a;

    // y控制量上限
    C.coeffRef(2 * horizon + i, i * control_dim_ + 1) = 1.0;
    cupp.coeffRef(2 * horizon + i, 0) = max_a;
    clow.coeffRef(2 * horizon + i, 0) = -max_a;

    // theta控制量上限
    C.coeffRef(3 * horizon + i, i * control_dim_ + control_dim_ - 1) = 1.0;
    cupp.coeffRef(3 * horizon + i, 0) = max_v;
    clow.coeffRef(3 * horizon + i, 0) = 0.0;
  }

  Cx.resize(horizon, state_dim_ * horizon);
  xup.resize(horizon, 1);
  xlow.resize(horizon, 1);
}

void Mpcc::Init(const Resample& referenceline, Eigen::SparseMatrix<double> state,
    const Config& config, nav_msgs::Path& ego_path) {
  if (init_flag) {
    output_index = 0;
    max_theta_ = referenceline.spline.getLength(); 
    optimal_theta.clear();
    stage.clear();
    // 初速度为0,初始轨迹为匀加速
    double v = 0.8 * config.theta_dot_upper_limit;
    double a = v / (Ts * horizon);
    double x, y, theta;
    for (int i = 0; i < horizon; i++) {
      theta = state.coeffRef(state_dim_ - 1, 0) + a * i * i * Ts * Ts / 2;
      auto pos_xy = referenceline.spline.getPostion(theta);
      auto dpos_xy = referenceline.spline.getDerivative(theta);
      double phi = atan2(dpos_xy(1), dpos_xy(0));

      std::vector<double> new_state{pos_xy[0], std::cos(phi) * a * i * Ts,
                                    pos_xy[1], std::sin(phi) * a * i * Ts, theta};
      // std::cout << new_state[1] << "," << new_state[3] <<std::endl;
      Stage stage_i(new_state);
      stage.emplace_back(stage_i);
      optimal_theta.emplace_back(theta);

      statePredict.coeffRef(0, i) = pos_xy[0];
      statePredict.coeffRef(1, i) = std::cos(phi) * a * i * Ts;
      statePredict.coeffRef(2, i) = pos_xy[1];
      statePredict.coeffRef(3, i) = std::sin(phi) * a * i * Ts;
      statePredict.coeffRef(4, i) = theta;
      ego_path.poses.resize(horizon);
      for (int i = 0; i < horizon; i++) {
        ego_path.poses[i].pose.position.x = statePredict.coeffRef(0, i);
        ego_path.poses[i].pose.position.y = statePredict.coeffRef(2, i);
        ego_path.poses[i].pose.position.z = config.hover_height;

        double psi = atan2(statePredict.coeffRef(3, i), statePredict.coeffRef(1, i));
        Eigen::Quaterniond q = Eigen::AngleAxisd(psi, ::Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(0.0, ::Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.0, ::Eigen::Vector3d::UnitX());
        ego_path.poses[i].pose.orientation.x = q.x();
        ego_path.poses[i].pose.orientation.y = q.y();
        ego_path.poses[i].pose.orientation.z = q.z();
        ego_path.poses[i].pose.orientation.w = q.w();
      }
      // ego_path_time = ros::Time::now();
    }
    init_flag = false;
  }
}

// 投影更新theta
void Mpcc::UpdateState(const Resample& referenceline,
    Eigen::SparseMatrix<double>& state) {
  double now_x = state.coeffRef(0, 0);
  double now_y = state.coeffRef(2, 0);
  double now_theta = referenceline.spline.porjectOnSpline(now_x, now_y);
  // state 与 optimal_theta要一致
  if (now_theta - optimal_theta[0] > 4.0 * max_theta_ / 5.0) {
    now_theta -= max_theta_;
  }
  if (optimal_theta[0] - now_theta > 4.0 * max_theta_ / 5.0) {
    now_theta += max_theta_;
  }
  state.coeffRef(state_dim_ - 1, 0) = now_theta;
  // std::cout << "lkjkl:" << now_theta << std::endl;
}

void Mpcc::RecedeOneHorizon(const Resample& referenceline) {
  // 修复在交界处theta优化错误的问题，如果回到开始，recede会出现错误
  optimal_theta.emplace_back(std::fmod(
    2 * (optimal_theta.back() + max_theta_) - optimal_theta.rbegin()[1], max_theta_));
  optimal_theta.erase(optimal_theta.begin());
  for (int i = 0; i < horizon - 1; i++) {
    if (optimal_theta[i + 1] - optimal_theta[i] + max_theta_ < 5.0) {
      optimal_theta[i + 1] += max_theta_;
    }
  }

  double x = 2 * stage.back().state[0] - stage.rbegin()[1].state[0];
  double y = 2 * stage.back().state[2] - stage.rbegin()[1].state[2];

  std::vector<double> new_state{x, stage.back().state[1], y, stage.back().state[3], optimal_theta.back()};

  Stage new_stage(new_state);

  for (int i = 0; i < horizon - 1; i++) {
    stage[i] = stage[i + 1];
  }
  stage[horizon - 1] = new_stage;
  // for (int i = 0;i<horizon;i++){
  //   std::cout <<"scscsc:" << optimal_theta[i] << std::endl;
  // }
  // std::cout <<optimal_theta[horizon - 1] << "," << stage[horizon - 1].state[4] << std::endl;
  // std::cout << std::endl;
}

void Mpcc::CalculateCost(const Resample& referenceline, const Config& config,
    Eigen::SparseMatrix<double> state, const nav_msgs::Path& ego_path,
    const nav_msgs::Path& obs_path, const nav_msgs::Odometry obs_odom, const Map& map) {
      //20 10 50
  double w_c = config.w_c;
  double w_l = config.w_l;
  double gamma = config.gamma;
  double w_u = config.w_u;
  double w_deltau = config.w_deltau;
  // double gamma = 0.001;
  Eigen::SparseMatrix<double> Qn;
  Eigen::SparseMatrix<double> qn;
  bool path_overlap_flag = false;
  if (obs_path.poses.size() > 0) {
    double ego_start_theta = referenceline.spline.porjectOnSpline(stage[0].state[0], stage[0].state[2]);
    double ego_end_theta = referenceline.spline.porjectOnSpline(stage[horizon - 1].state[0],
                                                                stage[horizon - 1].state[2]);
    double obs_start_theta = referenceline.spline.porjectOnSpline(obs_path.poses[0].pose.position.x,
                                                                  obs_path.poses[0].pose.position.y);
    double obs_end_theta = referenceline.spline.porjectOnSpline(obs_path.poses[horizon -1].pose.position.x,
                                                                obs_path.poses[horizon -1].pose.position.y);
    if (ego_start_theta > ego_end_theta) {
      ego_end_theta += max_theta_;
    }
    if (obs_start_theta > obs_end_theta) {
      obs_end_theta += max_theta_;
    }
    path_overlap_flag = (ego_end_theta >= obs_start_theta && ego_end_theta <= obs_end_theta) ||
      (obs_end_theta >= ego_start_theta && obs_end_theta <= ego_end_theta);
  }

  for (int i = 0; i < horizon; i++) {
    if (false) {
      double theta = std::fmod(optimal_theta[i], max_theta_);  // TODO 考虑闭环，theta跑一圈
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
      // 这里theta如果fmod就会优化出错，保证递增，在recede把这个修掉了
      for (int j = 0; j < state_dim_; j++) {
        X.coeffRef(j, 0) = stage[i].state[j];
      }
      // stage的theta经过recede不知道在哪里又被改了 TODO
      X.coeffRef(4, 0) = optimal_theta[i];
      std::vector<double> error(2, 0.0);
      Eigen::SparseMatrix<double> dEc(1, state_dim_);
      Eigen::SparseMatrix<double> dEl(1, state_dim_);
      GetErrorInfo(referenceline, stage[i], error, dEc, dEl);

      double gain = 1.0;
      if (i == horizon - 1) {
        gain = config.end_gain_rate;
      }
      double w_c_gain = 1.0;
      // // 加上效果不好，先不用
      // // 根据kappa调整横向系数提高安全性
      // if (i > 0 && i < horizon - 1) {
      //   std::vector<std::vector<double>> points{{stage[i - 1].state[0], stage[i - 1].state[2]},
      //                                           {stage[i].state[0], stage[i].state[2]},
      //                                           {stage[i + 1].state[0], stage[i + 1].state[2]}};
      //   double kappa = GetKappa(points);
      //   double kappa_lower_limit = 2.0;
      //   double kappa_upper_limit = 4.0;
      //   double w_c_gain_upper = 2.0;
      //   double w_c_gain_lower = 1.0;
        
      //   if (kappa >= kappa_lower_limit && kappa < kappa_upper_limit) {
      //     w_c_gain = (kappa - kappa_lower_limit) * (w_c_gain_upper - w_c_gain_lower)
      //       / (kappa_upper_limit - kappa_lower_limit) + w_c_gain_lower;
      //   } else if (kappa >= kappa_upper_limit) {
      //     w_c_gain = w_c_gain_upper;
      //   }
      //   // std::cout << kappa << "," << w_c_gain << std::endl;
      // }
      Qn = 2 * Eigen::SparseMatrix<double>(dEc.transpose()) * w_c_gain * gain * w_c * dEc +
           2 * Eigen::SparseMatrix<double>(dEl.transpose()) * gain * w_l * dEl;
      Eigen::SparseMatrix<double> c = dEc * X;
      Eigen::SparseMatrix<double> l = dEl * X;
      // std::cout << "err:" << X.coeffRef(4, 0) << std::endl;
      qn = 2 * gain * w_c_gain * w_c * (error[0] - c.coeffRef(0, 0)) *
               Eigen::SparseMatrix<double>(dEc.transpose()) +
           2 * gain * w_l * (error[1] - l.coeffRef(0, 0)) *
               Eigen::SparseMatrix<double>(dEl.transpose());
    }
    // obs penalty
    int collision_index = CheckCollision(ego_path, obs_path);
    // config.obs_penalty_valid = (collision_index != -1);
    double Sobs = config.Qobs;
    double x0 = config.obs_x;
    double y0 = config.obs_y;
    if (config.obs_penalty_valid) {
    // if (collision_index != -1) {
      x0 = (ego_path.poses[collision_index].pose.position.x + 
        obs_path.poses[collision_index].pose.position.x) / 2.0;
      y0 = (ego_path.poses[collision_index].pose.position.y + 
        obs_path.poses[collision_index].pose.position.y) / 2.0;
      std::vector<double> obst{x0, y0};
      std::vector<double> ego{stage[i].state[0], stage[i].state[2]};
      std::vector<double> coeff(6, 0.0);
      chance_constrains_set(coeff, obst, ego, config);

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
    double r = std::sqrt(std::pow(obs_odom.pose.pose.position.x - state.coeffRef(0, 0), 2) +
      std::pow(obs_odom.pose.pose.position.y - state.coeffRef(2, 0), 2));
    if (obs_path.poses.size() > 0 && path_overlap_flag) {
      int stage_index = GetStage(map, stage[i].state[0], stage[i].state[2]);
      if (stage_index >= 0) {
        std::vector<double> outer_vertical_point = GetVerticalPoint(map.outer_point_x_[stage_index],
                                                                    map.outer_point_y_[stage_index],
                                                                    map.outer_point_x_[stage_index + 1], 
                                                                    map.outer_point_y_[stage_index + 1],
                                                                    stage[i].state[0],
                                                                    stage[i].state[2]);
        std::vector<double> inner_vertical_point = GetVerticalPoint(map.inner_point_x_[stage_index],
                                                                    map.inner_point_y_[stage_index],
                                                                    map.inner_point_x_[stage_index + 1], 
                                                                    map.inner_point_y_[stage_index + 1],
                                                                    stage[i].state[0],
                                                                    stage[i].state[2]);
        double corridor_x, corridor_y;
        double S_corridor = 1.0 * config.theta_dot_upper_limit;
        if (config.group_index == 0) {
          corridor_x = outer_vertical_point[0];
          corridor_y = outer_vertical_point[1];
        } else {
          corridor_x = inner_vertical_point[0];
          corridor_y = inner_vertical_point[1];
        }
        Qn.coeffRef(0, 0) += S_corridor * 1.0;
        Qn.coeffRef(2, 2) += S_corridor * 1.0;
        qn.coeffRef(0, 0) += -S_corridor * corridor_x;
        qn.coeffRef(2, 0) += -S_corridor * corridor_y;
      }
      
      double S_muti = 0.3;
      double x2 = obs_path.poses[horizon - 1].pose.position.x;
      double y2 = obs_path.poses[horizon - 1].pose.position.y;
      double x1 = obs_path.poses[0].pose.position.x;
      double y1 = obs_path.poses[0].pose.position.y;
      double vert_x = y2 - y1;
      double vert_y = -(x2 - x1);
      double d2 = vert_x * vert_x + vert_y * vert_y;
      Qn.coeffRef(0, 0) -= S_muti * vert_x * vert_x / d2;
      Qn.coeffRef(0, 2) -= S_muti * vert_x * vert_y / d2;
      Qn.coeffRef(2, 2) -= S_muti * vert_y * vert_y / d2;
      Qn.coeffRef(2, 0) -= S_muti * vert_x * vert_y / d2;
      qn.coeffRef(0, 0) -= S_muti * (-x1 * vert_x * vert_x - vert_x * vert_y * y1) / d2;
      qn.coeffRef(2, 0) -= S_muti * (-y1 * vert_y * vert_x - vert_x * vert_y * x1) / d2;
    }
    sp::colMajor::setBlock(Q, Qn, state_dim_ * i, state_dim_ * i);
    sp::colMajor::setBlock(q, qn, state_dim_ * i, 0);
  }

  H = BBT * Q * BB + w_u * Q_u + w_deltau * Q_deltau;
  f = BBT * Q * AA * state + BBT * q - gamma * progress;
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

void Mpcc::SolveQp(const Resample& referenceline, const Map& map, const Config& config,
    Eigen::SparseMatrix<double> state, nav_msgs::Path& ego_path,
    const nav_msgs::Path& obs_path, const nav_msgs::Odometry obs_odom) {
  geometry_msgs::Point tmpPoint;
  geometry_msgs::Vector3 tmpVector;
  // 用来调pid
  std::vector<double> cmd(6, 0.0);
  if (config.circle_test_flag) {
    CircleTest(state, cmd, config);
    tmpPoint.x = cmd[0];
    tmpPoint.y = cmd[1];
    tmpPoint.z = config.hover_height;
    cmdMsg.position = tmpPoint;
    tmpVector.x = cmd[2];
    tmpVector.y = cmd[3];
    tmpVector.z = 0.0;
    cmdMsg.velocity = tmpVector;
    tmpVector.x = cmd[4];
    tmpVector.y = cmd[5];
    tmpVector.z = 0.0;
    cmdMsg.acceleration = tmpVector;
    cmdMsg.header.stamp = ros::Time::now();
    return;
  } 
  
  if (!mpcc_valid_flag_) {
    output_index = 0;
    auto pos_theta = referenceline.spline.getPostion(state.coeffRef(4, 0));
    tmpPoint.x = pos_theta(0);
    tmpPoint.y = pos_theta(1);
    tmpPoint.z = config.hover_height;
    cmdMsg.position = tmpPoint;
    tmpVector.x = 0.0;
    tmpVector.y = 0.0;
    tmpVector.z = 0.0;
    cmdMsg.velocity = tmpVector;
    tmpVector.x = 0.0;
    tmpVector.y = 0.0;
    tmpVector.z = 0.0;
    cmdMsg.acceleration = tmpVector;
    cmdMsg.header.stamp = ros::Time::now();
    return;
  }
  // 初始划轨迹
  Init(referenceline, state, config, ego_path);
  // 更新一步
  RecedeOneHorizon(referenceline);
  // 设置约束
  SetConstrains(referenceline, map, state, config, ego_path, obs_path);
  // 设置QP矩阵
  CalculateCost(referenceline, config, state, ego_path, obs_path, obs_odom, map);
  // 更新QP矩阵
  osqpInterface.updateMatrices(H, f, A, b, C, clow, cupp, ul, uu);
  // QP求解
  osqpInterface.solveQP();
  auto solveStatus = osqpInterface.solveStatus();
  auto solPtr = osqpInterface.solPtr();

  if (solveStatus == OSQP_SOLVED) {
    // 有解就用第一个
    output_index = 0;
    optimal_theta.clear();
    x_horizon.clear();
    y_horizon.clear();
    theta_x_.clear();
    theta_y_.clear();
    Eigen::SparseMatrix<double> state_horizon = state;
    for (int i = 0; i < horizon; i++) {
      for (int j = 0; j < control_dim_; j++) {
        inputPredict.coeffRef(j, i) = solPtr->x[i * control_dim_ + j];
      }
      // 计算对应优化状态
      state_horizon = Ad * state_horizon + Bd * inputPredict.col(i);
      // std::cout << "asasas: " << state_horizon.coeffRef(state_dim_ - 1, 0) << "," << inputPredict.coeffRef(2, i) << std::endl;
      state_horizon.coeffRef(state_dim_ - 1, 0) = std::fmod(state_horizon.coeffRef(state_dim_ - 1, 0), max_theta_);
      sp::colMajor::setCols(statePredict, state_horizon, i);

      // 更新最优轨迹
      optimal_theta.emplace_back(state_horizon.coeffRef(state_dim_ - 1, 0));
      for (int j = 0; j < state_dim_; j++) {
        stage[i].state[j] = state_horizon.coeffRef(j, 0);
      }

      // update for plot
      x_horizon.emplace_back(statePredict.coeffRef(0, i));
      y_horizon.emplace_back(statePredict.coeffRef(2, i));
      auto pos = referenceline.spline.getPostion(statePredict.coeffRef(4, i));
      theta_x_.emplace_back(pos(0));
      theta_y_.emplace_back(pos(1));

      // 加速度限制加上，安全检查关掉
      // double max_a = 9.8 * std::tan(config.angle_upper_limit * PI / 180.0);
      // if (std::fabs(inputPredict.coeffRef(0, output_index)) > max_a ||
      //     std::fabs(inputPredict.coeffRef(0, output_index)) > max_a) {
      //   mpcc_valid_flag_ = false;
      // }
    }
    // std::cout << std::endl;
  } else {
    // 无解, 就用上一帧结果
    output_index++;
    // 连续五帧无解，mpcc -> invalid
    if (output_index > 5) {
      mpcc_valid_flag_ = false;
    }
    std::cout << "no solution" << std::endl;
  }
  // 防止越界
  output_index = std::min(output_index, horizon - 1);
  // 控制指令
  tmpPoint.x = stage[output_index].state[0];
  tmpPoint.y = stage[output_index].state[2];
  tmpPoint.z = config.hover_height;
  cmdMsg.position = tmpPoint;
  tmpVector.x = stage[output_index].state[1];
  tmpVector.y = stage[output_index].state[3];
  tmpVector.z = 0.0;
  cmdMsg.velocity = tmpVector;
  tmpVector.x = inputPredict.coeffRef(0, output_index);
  tmpVector.y = inputPredict.coeffRef(1, output_index);
  tmpVector.z = 0.0;
  cmdMsg.acceleration = tmpVector;
  cmdMsg.header.stamp = ros::Time::now();
  // max_cmd_a = std::max({max_cmd_a, std::fabs(inputPredict.coeffRef(0, output_index)),
  //   std::fabs(inputPredict.coeffRef(1, output_index))});
  // std::cout << output_index <<  std::endl;
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

bool Mpcc::InCorridorRange(const Map& map, double x, double y) {
  for (int i = 0; i < map.outer_point_x_.size() - 1; i++) {
    std::vector<std::vector<double>> rec{
      {map.outer_point_x_[i], map.outer_point_y_[i]},
      {map.outer_point_x_[i + 1], map.outer_point_y_[i + 1]},
      {map.inner_point_x_[i + 1], map.inner_point_y_[i + 1]},
      {map.inner_point_x_[i], map.inner_point_y_[i]}
    };
    if (InQuad(rec, x, y)) {
      return true;
    }
  }
  return false;
}

double Mpcc::GetKappa(std::vector<std::vector<double>> points) {
  double a = sqrt(std::pow(points[0][0] - points[1][0], 2) + std::pow(points[0][1] - points[1][1], 2));
  double b = sqrt(std::pow(points[1][0] - points[2][0], 2) + std::pow(points[1][1] - points[2][1], 2));
  double c = sqrt(std::pow(points[2][0] - points[0][0], 2) + std::pow(points[2][1] - points[0][1], 2));
  // 海伦公式
  double p = (a + b + c) / 2.0;
  double s = std::sqrt(p * (p - a) * (p - b) * (p - c));
  double r = a * b * c / (4 * s);
  return 1 / r;
}
// cmd : x, y , vx, vy, ax, ay
void Mpcc::CircleTest(Eigen::SparseMatrix<double> state, std::vector<double>& cmd, const Config& config) {
  double r = 1.0;
  double start_point_range = 0.1;
  if (!start_test_flag) {
    if (std::pow(state.coeffRef(0, 0) - r, 2) + std::pow(state.coeffRef(2, 0) - 0, 2)
        > start_point_range * start_point_range) {
      cmd[0] = r;
      cmd[1] = 0.0;
      cmd[2] = 0.0;
      cmd[3] = 0.0;
      cmd[4] = 0.0;
      cmd[5] = 0.0;
      return;
    } else {
      start_test_flag = true;
      start_test_time = ros::Time::now();
    }
  }
  
  double v = config.circle_test_v;
  double omega = v / r;
  int count = 1;
  double t = (ros::Time::now() - start_test_time).toSec();
  if (v * t < 2 * PI * r * static_cast<double>(count)) {
    cmd[0] = r * std::cos(omega * t);
    cmd[1] = r * std::sin(omega * t);
    cmd[2] = -r * omega * std::sin(omega * t);
    cmd[3] = r * omega * std::cos(omega * t);
    cmd[4] = -r * omega * omega * std::cos(omega * t);
    cmd[5] = -r * omega * omega * std::sin(omega * t);
  } else {
    cmd[0] = r;
    cmd[1] = 0.0;
    cmd[2] = 0.0;
    cmd[3] = 0.0;
    cmd[4] = 0.0;
    cmd[5] = 0.0;
  }
}