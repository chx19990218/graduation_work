// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "map.h"
#include "mpcc.h"
#include "obstacle.h"

// TODO ：维度逐渐增加

void Mpcc::SetConstrains(const Resample& referenceline, const Map& map,
    Eigen::SparseMatrix<double> state) {
  // 走廊边界限制
  Cx.resize(horizon, state_dim_ * horizon);
  xup.resize(horizon, 1);
  xlow.resize(horizon, 1);

  for (int i = 0; i < horizon; i++) {
    double x = stage[i].state[0];
    double y = stage[i].state[2];
    std::vector<double> border = GetPointBorderConstrain(map, x, y, referenceline);
    // std::cout << border[0] << "," << border[1] << "," << border[2] << "," << border[3] << std::endl;
    
    double numer = -(border[0] - border[2]);
    double denom = (border[1] - border[3]);
    double dbmax = std::max(numer * border[0] - denom * border[1], numer * border[2] - denom * border[3]);
    double dbmin = std::min(numer * border[0] - denom * border[1], numer * border[2] - denom * border[3]);

    Cx.coeffRef(i, i * state_dim_ + 0) = numer;
    Cx.coeffRef(i, i * state_dim_ + 2) = -denom;
    xup.coeffRef(i, 0) = dbmax;
    xlow.coeffRef(i, 0) = dbmin;
  }
  // std::cout << std::endl;
  // 速度限制
  // TODO

  C = Eigen::SparseMatrix<double>(Cx * BB);
  cupp = xup - Eigen::SparseMatrix<double>(Cx * AA * state);
  clow = xlow - Eigen::SparseMatrix<double>(Cx * AA * state);

  // 加速度/角度,里程速度限制限制
  double max_attitude = 45.0 * PI / 180.0;
  double max_a = 9.8 * std::tan(max_attitude);
  double max_v = 1.0;
  for (int i = 0; i < horizon; i++) {
    // x轴控制量上限
    Eigen::SparseMatrix<double> Ck1(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk1(1, 1);
    Eigen::SparseMatrix<double> xlowk1(1, 1);
    Ck1.coeffRef(0, i * control_dim_ + 0) = 1.0;
    xupk1.coeffRef(0, 0) = max_a;
    xlowk1.coeffRef(0, 0) = -max_a;
    sp::colMajor::addRows(C, Ck1);
    sp::colMajor::addRows(cupp, xupk1);
    sp::colMajor::addRows(clow, xlowk1);

    // y控制量上限
    Eigen::SparseMatrix<double> Ck2(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk2(1, 1);
    Eigen::SparseMatrix<double> xlowk2(1, 1);
    Ck2.coeffRef(0, i * control_dim_ + 1) = 1.0;
    xupk2.coeffRef(0, 0) = max_a;
    xlowk2.coeffRef(0, 0) = -max_a;
    sp::colMajor::addRows(C, Ck2);
    sp::colMajor::addRows(cupp, xupk2);
    sp::colMajor::addRows(clow, xlowk2);

    // theta控制量上限
    Eigen::SparseMatrix<double> Ck3(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk3(1, 1);
    Eigen::SparseMatrix<double> xlowk3(1, 1);
    Ck3.coeffRef(0, i * control_dim_ + control_dim_ - 1) = 1.0;
    xupk3.coeffRef(0, 0) = max_v;
    xlowk3.coeffRef(0, 0) = 0.0;
    sp::colMajor::addRows(C, Ck3);
    sp::colMajor::addRows(cupp, xupk3);
    sp::colMajor::addRows(clow, xlowk3);
  }
}

std::vector<double> Mpcc::GetPointBorderConstrain(const Map& map, double x,
                                                  double y, const Resample& referenceline) {
  int stage_index = GetStage(map, x, y);

  std::vector<double> res(4);
  if (stage_index >= 0) {
    // 进入障碍区，后续更新进入条件
    if (y > 2.0 && y < 4.0 && x < 1.0) {
      
      res = GetBorder(x, y);
      
    } else { // 没进入障碍区
      // 直接用参考线向两侧拓展一个半径
      double s = referenceline.spline.porjectOnSpline(x, y);
      auto center_p = referenceline.spline.getPostion(s);
      auto center_v = referenceline.spline.getDerivative(s);
      double r = 0.5;
      double x_outer = center_p(0) + r * (-center_v(1));
      double y_outer = center_p(1) + r * (center_v(0));
      double x_inner = center_p(0) + r * (center_v(1));
      double y_inner = center_p(1) + r * (-center_v(0));
      res[0] = x_outer;
      res[1] = y_outer;
      res[2] = x_inner;
      res[3] = y_inner;
      // 向直角转弯的走廊边界作垂线获得边界
      // double x1 = map.outer_point_x_[stage_index];
      // double y1 = map.outer_point_y_[stage_index];
      // double x2 = map.outer_point_x_[stage_index + 1];
      // double y2 = map.outer_point_y_[stage_index + 1];
      // std::vector<double> outer_vertical_point = GetVerticalPoint(x1, y1, x2, y2, x, y);
      // res[0] = outer_vertical_point[0];
      // res[1] = outer_vertical_point[1];
      // x1 = map.inner_point_x_[stage_index];
      // y1 = map.inner_point_y_[stage_index];
      // x2 = map.inner_point_x_[stage_index + 1];
      // y2 = map.inner_point_y_[stage_index + 1];
      // std::vector<double> inner_vertical_point = GetVerticalPoint(x1, y1, x2, y2, x, y);
      // res[2] = inner_vertical_point[0];
      // res[3] = inner_vertical_point[1];
    }
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

// 按顺时针/逆时针传入
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