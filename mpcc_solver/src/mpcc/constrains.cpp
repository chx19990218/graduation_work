// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "map.h"
#include "mpcc.h"
#include "obstacle.h"
#include "config.h"

// TODO ：维度逐渐增加

void Mpcc::SetConstrains(const Resample& referenceline, const Map& map,
    Eigen::SparseMatrix<double> state, const Config& config) {
  // 走廊边界限制
  Cx.resize(horizon, state_dim_ * horizon);
  xup.resize(horizon, 1);
  xlow.resize(horizon, 1);
  for (int i = 0; i < horizon; i++) {
    double x = stage[i].state[0];
    double y = stage[i].state[2];
    std::vector<double> border = GetPointBorderConstrain(map, x, y, referenceline, config);
    // std::cout << border[0] << "," << border[1] << "," << border[2] << "," << border[3] << std::endl;
    double numer = -(border[0] - border[2]);
    double denom = (border[1] - border[3]);
    double dbmax = std::max(numer * border[0] - denom * border[1], numer * border[2] - denom * border[3]);
    double dbmin = std::min(numer * border[0] - denom * border[1], numer * border[2] - denom * border[3]);

    Cx.coeffRef(i, i * state_dim_ + 0) = numer;
    Cx.coeffRef(i, i * state_dim_ + 2) = -denom;
    xup.coeffRef(i, 0) = dbmax;
    xlow.coeffRef(i, 0) = dbmin;
    // xup.coeffRef(i, 0) = 100000;
    // xlow.coeffRef(i, 0) = -100000;
  }
  // std::cout << std::endl;
  // 速度限制
  // TODO

  // addRows 耗时很大，换种方式定义
  Eigen::SparseMatrix<double> C_temp = Eigen::SparseMatrix<double>(Cx * BB);
  Eigen::SparseMatrix<double> cupp_temp = xup - Eigen::SparseMatrix<double>(Cx * AA * state);
  Eigen::SparseMatrix<double> clow_temp = xlow - Eigen::SparseMatrix<double>(Cx * AA * state);

  C.resize(C_temp.rows() + horizon, C_temp.cols());
  cupp.resize(cupp_temp.rows() + horizon, 1);
  clow.resize(clow_temp.rows() + horizon, 1);

  sp::colMajor::setBlock(C, C_temp, 0, 0);
  sp::colMajor::setBlock(cupp, cupp_temp, 0, 0);
  sp::colMajor::setBlock(clow, clow_temp, 0, 0);

  // 加速度/角度,里程速度限制限制
  double max_attitude = config.angle_upper_limit * PI / 180.0;
  double max_a = 9.8 * std::tan(max_attitude);
  double max_v = config.theta_dot_upper_limit;
  for (int i = 0; i < horizon; i++) {
    // // x轴控制量上限
    // C.coeffRef(C_temp.rows() + i, i * control_dim_ + 0) = 1.0;
    // cupp.coeffRef(cupp_temp.rows() + i, 0) = max_a;
    // clow.coeffRef(clow_temp.rows() + i, 0) = -max_a;

    // // y控制量上限
    // C.coeffRef(C_temp.rows() + horizon + i, i * control_dim_ + 1) = 1.0;
    // cupp.coeffRef(cupp_temp.rows() + horizon + i, 0) = max_a;
    // clow.coeffRef(clow_temp.rows() + horizon + i, 0) = -max_a;

    // theta控制量上限
    C.coeffRef(C_temp.rows() + i, i * control_dim_ + control_dim_ - 1) = 1.0;
    cupp.coeffRef(cupp_temp.rows() + i, 0) = max_v;
    clow.coeffRef(clow_temp.rows() + i, 0) = 0.0;
  }
}

std::vector<double> Mpcc::GetPointBorderConstrain(const Map& map, double x,
                                                  double y, const Resample& referenceline,
                                                  const Config& config) {
  
  int stage_index = GetStage(map, x, y);
  std::vector<double> res(4);
  if (stage_index >= 0) {
    // 把判断是否在障碍范围内提出来，dp关掉就不计算
    bool get_into_obstacle_flag = false;
    if (config.enable_dp_flag) {
      double horizon_dist = horizon * config.theta_dot_upper_limit * Ts + 0.1;
      double obs_y = (obstacle_pos_[0][1] + obstacle_pos_[2][1]) / 2.0;
      double obs_x = (obstacle_pos_[0][0] + obstacle_pos_[2][0]) / 2.0;
      double obs_x_r = std::fabs(obstacle_pos_[0][0] - obstacle_pos_[2][0]) / 2.0;
      get_into_obstacle_flag = y < obs_y + horizon_dist * config.dp_length_rate &&
        y > obs_y - horizon_dist * config.dp_length_rate && x < obs_x + obs_x_r &&
        x > obs_x - obs_x_r;

      // projection操作耗时很大
      // double obs_size = std::max(std::fabs(obstacle_pos_[0][0] - obstacle_pos_[2][0]),
      //   std::fabs(obstacle_pos_[0][1] - obstacle_pos_[2][1]));
      // double now_theta = referenceline.spline.porjectOnSpline(x, y);
      // double obs_x = (obstacle_pos_[0][0] + obstacle_pos_[2][0]) / 2.0;
      // double obs_y = (obstacle_pos_[0][1] + obstacle_pos_[2][1]) / 2.0;
      // double obs_theta = referenceline.spline.porjectOnSpline(obs_x, obs_y);
      // get_into_obstacle_flag = std::fabs(obs_theta - now_theta) <
      //   std::max(horizon_dist * config.dp_length_rate, obs_size);
    }

    if (config.enable_dp_flag && get_into_obstacle_flag) {// 进入障碍区
      res = GetBorder(x, y);
    } else { // 没进入障碍区
      // 方式一：直接用参考线向两侧拓展一个半径
      // double s = referenceline.spline.porjectOnSpline(x, y);
      // auto center_p = referenceline.spline.getPostion(s);
      // auto center_v = referenceline.spline.getDerivative(s);
      // double r = 0.5;
      // double x_outer = center_p(0) + r * (-center_v(1));
      // double y_outer = center_p(1) + r * (center_v(0));
      // double x_inner = center_p(0) + r * (center_v(1));
      // double y_inner = center_p(1) + r * (-center_v(0));
      // res[0] = x_outer;
      // res[1] = y_outer;
      // res[2] = x_inner;
      // res[3] = y_inner;

      // 方式二：向直角转弯的走廊边界作垂线获得边界
      double x1 = map.outer_point_x_[stage_index];
      double y1 = map.outer_point_y_[stage_index];
      double x2 = map.outer_point_x_[stage_index + 1];
      double y2 = map.outer_point_y_[stage_index + 1];
      std::vector<double> outer_vertical_point = GetVerticalPoint(x1, y1, x2, y2, x, y);
      res[0] = outer_vertical_point[0];
      res[1] = outer_vertical_point[1];
      x1 = map.inner_point_x_[stage_index];
      y1 = map.inner_point_y_[stage_index];
      x2 = map.inner_point_x_[stage_index + 1];
      y2 = map.inner_point_y_[stage_index + 1];
      std::vector<double> inner_vertical_point = GetVerticalPoint(x1, y1, x2, y2, x, y);
      res[2] = inner_vertical_point[0];
      res[3] = inner_vertical_point[1];
    }
  } else {
    res[0] = 100000;
    res[1] = 100000;
    res[2] = -100000;
    res[3] = -100000;
  }
  return res;
}

// 求取点到边的垂线交点
// point1, point2是边, point3是外点, point4是待求点
std::vector<double> Mpcc::GetVerticalPoint(double x1, double y1, double x2,
                                           double y2, double x3, double y3) {
  double a11 = x1 - x2;
  double a12 = y1 - y2;
  double b1 = x3 * (x1 - x2) + y3 * (y1 - y2);
  double a21 = y1 - y2;
  double a22 = -(x1 - x2);
  double b2 = x2 * (y1 - y2) - y2 * (x1 - x2);
  double D = a11 * a22 - a12 * a21;
  double D1 = b1 * a22 - b2 * a12;
  double D2 = a11 * b2 - b1 * a21;

  double x4 = D1 / D;
  double y4 = D2 / D;
  return std::vector<double> {x4, y4};
}

std::vector<double> Mpcc::GetBorder(double now_x, double now_y) {
  std::vector<double> border(4, 0.0);
  int index = 0;
  if (now_y < optimal_path_y[0]) {
    index = 0;
    border[0] = left_border_x[0];
    border[1] = left_border_y[0];
    border[2] = right_border_x[0];
    border[3] = right_border_y[0];
  } else if (now_y >= optimal_path_y.back()) {
    index = optimal_path_y.size() - 1;
    border[0] = left_border_x.back();
    border[1] = left_border_y.back();
    border[2] = right_border_x.back();
    border[3] = right_border_y.back();
  } else {
    for (int i = 1; i < optimal_path_y.size(); i++) {
      index = i;
      if (now_y >= optimal_path_y[i - 1] && now_y < optimal_path_y[i]) {
        double rate = (now_y - optimal_path_y[i - 1]) /
                      (optimal_path_y[i] - optimal_path_y[i - 1]);
        border[0] = left_border_x[i - 1] +
                    (left_border_x[i] - left_border_x[i - 1]) * rate;
        border[1] = left_border_y[i - 1] +
                    (left_border_y[i] - left_border_y[i - 1]) * rate;
        border[2] = right_border_x[i - 1] +
                    (right_border_x[i] - right_border_x[i - 1]) * rate;
        border[3] = right_border_y[i - 1] +
                    (right_border_y[i] - right_border_y[i - 1]) * rate;
        break;
      }
    }
  }
  // std::cout << border[0] << "," << border[1] << "," << border[2] << "," << border[3] << std::endl;
  return border;
}

int Mpcc::GetStage(const Map& map, double x, double y) {
  for (int i = 0; i < map.stage.size(); i++) {
    auto rec = map.stage[i];
    if (InRec(rec, x, y)) {
      return i;
    }
  }
  return -1;
}

// 按顺时针/逆时针传入 矩形
bool Mpcc::InRec(std::vector<std::vector<double>>& rec, double x, double y) {
  // 在边上也算
  bool flag1 = ((rec[0][0] - x) * (rec[1][0] - rec[0][0]) +
                (rec[0][1] - y) * (rec[1][1] - rec[0][1])) <= 0;
  bool flag2 = ((rec[1][0] - x) * (rec[2][0] - rec[1][0]) +
                (rec[1][1] - y) * (rec[2][1] - rec[1][1])) <= 0;
  bool flag3 = ((rec[2][0] - x) * (rec[3][0] - rec[2][0]) +
                (rec[2][1] - y) * (rec[3][1] - rec[2][1])) <= 0;
  bool flag4 = ((rec[3][0] - x) * (rec[0][0] - rec[3][0]) +
                (rec[3][1] - y) * (rec[0][1] - rec[3][1])) <= 0;
  return flag1 & flag2 & flag3 & flag4;
}

// 按顺时针/逆时针传入 四边形
bool Mpcc::InQuad(std::vector<std::vector<double>>& rec, double x, double y) {
  double v1_x = rec[1][0] - rec[0][0];
  double v2_x = rec[2][0] - rec[1][0];
  double v3_x = rec[3][0] - rec[2][0];
  double v4_x = rec[0][0] - rec[3][0];

  double v1_y = rec[1][1] - rec[0][1];
  double v2_y = rec[2][1] - rec[1][1];
  double v3_y = rec[3][1] - rec[2][1];
  double v4_y = rec[0][1] - rec[3][1];

  double ego1_x = x - rec[0][0];
  double ego2_x = x - rec[1][0];
  double ego3_x = x - rec[2][0];
  double ego4_x = x - rec[3][0];

  double ego1_y = y - rec[0][1];
  double ego2_y = y - rec[1][1];
  double ego3_y = y - rec[2][1];
  double ego4_y = y - rec[3][1];
  
  double k1 = v1_x * ego1_y - v1_y * ego1_x;
  double k2 = v2_x * ego2_y - v2_y * ego2_x;
  double k3 = v3_x * ego3_y - v3_y * ego3_x;
  double k4 = v4_x * ego4_y - v4_y * ego4_x;

  return (k1 > 0 && k2 > 0 && k3 > 0 && k4 > 0)
    || (k1 < 0 && k2 < 0 && k3 < 0 && k4 < 0);
}