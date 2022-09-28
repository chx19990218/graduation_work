// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "map.h"
#include "mpcc.h"

// TODO ：维度逐渐增加

void Mpcc::SetConstrains(const Resample& referenceline, const Map& map) {
  // 走廊边界限制
  Cx.resize(2 * horizon, state_dim_ * horizon);
  xup.resize(2 * horizon, 1);
  xlow.resize(2 * horizon, 1);
  std::vector<double> vec_outer, vec_inner, vec_outer_vertical,
      vec_inner_vertical;
  int stage_index;
  for (int i = 0; i < horizon; i++) {
    auto pos = referenceline.spline.getPostion(optimal_theta[i]);
    double x = pos[0];
    double y = pos[1];
    stage_index = GetStage(map, x, y);
    if (stage_index >= 0) {
      vec_outer = std::vector<double>{
          map.outer_point_x_[stage_index + 1] - map.outer_point_x_[stage_index],
          map.outer_point_y_[stage_index + 1] -
              map.outer_point_y_[stage_index]};
      vec_inner = std::vector<double>{
          map.inner_point_x_[stage_index] - map.inner_point_x_[stage_index + 1],
          map.inner_point_y_[stage_index] -
              map.inner_point_y_[stage_index + 1]};
      vec_outer_vertical = std::vector<double>{-vec_outer[1], vec_outer[0]};
      vec_inner_vertical = std::vector<double>{-vec_inner[1], vec_inner[0]};

      Cx.coeffRef(2 * i, 0 + i * state_dim_) = vec_outer_vertical[0];
      Cx.coeffRef(2 * i, 2 + i * state_dim_) = vec_outer_vertical[1];
      xup.coeffRef(2 * i, 0) =
          map.outer_point_x_[stage_index] * vec_outer_vertical[0] +
          map.outer_point_y_[stage_index] * vec_outer_vertical[1];
      Cx.coeffRef(2 * i + 1, 0 + i * state_dim_) = vec_inner_vertical[0];
      Cx.coeffRef(2 * i + 1, 2 + i * state_dim_) = vec_inner_vertical[1];
      xup.coeffRef(2 * i + 1, 0) =
          map.inner_point_x_[stage_index+1] * vec_inner_vertical[0] +
          map.inner_point_y_[stage_index+1] * vec_inner_vertical[1];
    }
  }
  // std::cout<<vec_outer_vertical[0]<<","<<vec_outer_vertical[1]<<","<<vec_inner_vertical[0]<<","<<vec_inner_vertical[1]<<std::endl;

  // 速度限制
  // TODO

  C = Eigen::SparseMatrix<double>(Cx * BB);
  cupp = xup - Eigen::SparseMatrix<double>(Cx * AA * state);

  // 加速度/角度,里程速度限制限制
  double max_attitude = 45.0 * PI / 180.0;
  double max_a = std::tan(max_attitude);
  double max_v = 10.0;
  for (int i = 0; i < horizon; i++) {
    // x控制量上限
    Eigen::SparseMatrix<double> Ck1(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk1(1, 1);
    Ck1.coeffRef(0, i * control_dim_ + 0) = 1.0;
    xupk1.coeffRef(0, 0) = max_a;
    sp::colMajor::addRows(C, Ck1);
    sp::colMajor::addRows(cupp, xupk1);

    // y控制量上限
    Eigen::SparseMatrix<double> Ck2(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk2(1, 1);
    Ck2.coeffRef(0, i * control_dim_ + 1) = 1.0;
    xupk2.coeffRef(0, 0) = max_a;
    sp::colMajor::addRows(C, Ck2);
    sp::colMajor::addRows(cupp, xupk2);

    // theta控制量上限
    Eigen::SparseMatrix<double> Ck3(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk3(1, 1);
    Ck3.coeffRef(0, i * control_dim_ + control_dim_ - 1) = 1.0;
    xupk3.coeffRef(0, 0) = max_v;
    sp::colMajor::addRows(C, Ck3);
    sp::colMajor::addRows(cupp, xupk3);

    // x控制量下限
    Eigen::SparseMatrix<double> Ck4(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk4(1, 1);
    Ck4.coeffRef(0, i * control_dim_ + 0) = -1.0;
    xupk4.coeffRef(0, 0) = max_a;
    sp::colMajor::addRows(C, Ck4);
    sp::colMajor::addRows(cupp, xupk4);

    // y控制量下限
    Eigen::SparseMatrix<double> Ck5(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk5(1, 1);
    Ck5.coeffRef(0, i * control_dim_ + 1) = -1.0;
    xupk5.coeffRef(0, 0) = max_a;
    sp::colMajor::addRows(C, Ck5);
    sp::colMajor::addRows(cupp, xupk5);

    // theta控制量下限
    Eigen::SparseMatrix<double> Ck6(1, horizon * control_dim_);
    Eigen::SparseMatrix<double> xupk6(1, 1);
    Ck6.coeffRef(0, i * control_dim_ + control_dim_ - 1) = -1.0;
    xupk6.coeffRef(0, 0) = 0.0;
    sp::colMajor::addRows(C, Ck6);
    sp::colMajor::addRows(cupp, xupk6);
  }
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