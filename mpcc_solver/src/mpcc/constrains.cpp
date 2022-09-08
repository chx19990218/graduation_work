// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "map.h"
#include "mpcc.h"

void Mpcc::SetConstrains(const Resample& referenceline, const Map& map) {
  Cx.resize(2 * horizon, state_dim_);
  xup.resize(2 * horizon, 1);
  xlow.resize(2 * horizon, 1);
  for (int i = 0; i < horizon; i++) {
    auto pos = referenceline.spline.getPostion(optimal_theta[i]);
    double x = pos[0];
    double y = pos[1];
    int stage_index = GetStage(map, x, y);
    if (stage_index > 0) {
      std::vector<double> vec_outer = std::vector<double>{
          map.outer_point_x_[stage_index + 1] - map.outer_point_x_[stage_index],
          map.outer_point_y_[stage_index + 1] -
              map.outer_point_y_[stage_index]};
      std::vector<double> vec_inner = std::vector<double>{
          map.outer_point_x_[stage_index] - map.outer_point_x_[stage_index + 1],
          map.outer_point_y_[stage_index] -
              map.outer_point_y_[stage_index + 1]};
      std::vector<double> vec_outer_vertical =
          std::vector<double>{-vec_outer[1], vec_outer[0]};
      std::vector<double> vec_inner_vertical =
          std::vector<double>{-vec_inner[1], vec_inner[0]};
      Cx.coeffRef(2 * i, 0) = vec_outer_vertical[0];
      Cx.coeffRef(2 * i, 1) = vec_outer_vertical[1];
      xup.coeffRef(2 * i, 0) =
          map.outer_point_x_[stage_index] * vec_outer_vertical[0] +
          map.outer_point_y_[stage_index] * vec_outer_vertical[1];
      Cx.coeffRef(2 * i + 1, 0) = vec_inner_vertical[0];
      Cx.coeffRef(2 * i + 1, 1) = vec_inner_vertical[1];
      xup.coeffRef(2 * i + 1, 0) =
          map.inner_point_x_[stage_index] * vec_inner_vertical[0] +
          map.inner_point_y_[stage_index] * vec_inner_vertical[1];
    }
  }
}

int Mpcc::GetStage(const Map& map, double x, double y) {
  int size = map.center_point_.size();
  std::vector<double> now_outer_point(2, 0.0);
  std::vector<double> now_inner_point(2, 0.0);
  std::vector<double> next_outer_point(2, 0.0);
  std::vector<double> next_inner_point(2, 0.0);
  for (int i = 0; i < size; i++) {
    now_outer_point[0] = map.outer_point_x_[i];
    now_outer_point[1] = map.outer_point_y_[i];
    now_inner_point[0] = map.inner_point_x_[i];
    now_inner_point[1] = map.inner_point_y_[i];
    next_outer_point[0] = map.outer_point_x_[i + 1];
    next_outer_point[1] = map.outer_point_y_[i + 1];
    next_inner_point[0] = map.inner_point_x_[i + 1];
    next_inner_point[1] = map.inner_point_y_[i + 1];
    std::vector<std::vector<double>> vec;
    GenRec(vec, now_outer_point, now_inner_point, next_outer_point,
           next_inner_point);
    if (InRec(vec, x, y)) {
      return i;
    }
  }
  return -1;
}

// 根据四边形生成矩形，按顺序添加
void Mpcc::GenRec(std::vector<std::vector<double>>& vec,
                  const std::vector<double>& now_outer_point,
                  const std::vector<double>& now_inner_point,
                  const std::vector<double>& next_outer_point,
                  const std::vector<double>& next_inner_point) {
  vec.clear();
  bool now_flag, next_flag;
  if (std::abs(now_outer_point[0] - next_outer_point[0]) >
      std::abs(now_outer_point[1] - next_outer_point[1])) {
    // 横
    if ((next_outer_point[0] - now_outer_point[0]) *
            (now_outer_point[0] - now_inner_point[0]) >
        0) {
      vec.emplace_back(now_inner_point);
      now_flag = true;
    } else {
      vec.emplace_back(now_outer_point);
      now_flag = false;
    }
    if ((next_outer_point[0] - now_outer_point[0]) *
            (next_outer_point[0] - next_inner_point[0]) >
        0) {
      vec.emplace_back(next_inner_point);
      next_flag = true;
    } else {
      vec.emplace_back(next_outer_point);
      next_flag = false;
    }
    if (now_flag == next_flag) {
      // 一条线
      if (now_flag) {
        // 里
        vec.emplace_back(
            std::vector<double>{next_inner_point[0], now_outer_point[1]});
        vec.emplace_back(
            std::vector<double>{now_inner_point[0], now_outer_point[1]});
      } else {
        // 外
        vec.emplace_back(
            std::vector<double>{next_outer_point[0], now_inner_point[1]});
        vec.emplace_back(
            std::vector<double>{now_outer_point[0], now_inner_point[1]});
      }
    } else {
      // 对角
      if (now_flag) {
        // 里
        vec.insert(vec.begin() + 1, std::vector<double>{now_inner_point[0],
                                                        next_outer_point[1]});
        vec.emplace_back(
            std::vector<double>{next_outer_point[0], now_inner_point[1]});
      } else {
        // 外
        vec.insert(vec.begin() + 1, std::vector<double>{now_outer_point[0],
                                                        next_inner_point[1]});
        vec.emplace_back(
            std::vector<double>{next_inner_point[0], now_outer_point[1]});
      }
    }
  } else {
    // 纵
    if ((next_outer_point[1] - now_outer_point[1]) *
            (now_outer_point[1] - now_inner_point[1]) >
        0) {
      vec.emplace_back(now_inner_point);
      now_flag = true;
    } else {
      vec.emplace_back(now_outer_point);
      now_flag = false;
    }
    if ((next_outer_point[1] - now_outer_point[1]) *
            (next_outer_point[1] - next_inner_point[1]) >
        0) {
      vec.emplace_back(next_inner_point);
      next_flag = true;
    } else {
      vec.emplace_back(next_outer_point);
      next_flag = false;
    }
    if (now_flag == next_flag) {
      // 一条线
      if (now_flag) {
        // 里
        vec.emplace_back(
            std::vector<double>{now_outer_point[0], next_inner_point[1]});
        vec.emplace_back(
            std::vector<double>{now_outer_point[0], now_inner_point[1]});
      } else {
        // 外
        vec.emplace_back(
            std::vector<double>{now_inner_point[0], next_outer_point[1]});
        vec.emplace_back(
            std::vector<double>{now_inner_point[0], now_outer_point[1]});
      }
    } else {
      // 对角
      if (now_flag) {
        // 里
        vec.insert(vec.begin() + 1, std::vector<double>{now_inner_point[0],
                                                        next_outer_point[1]});
        vec.emplace_back(
            std::vector<double>{next_outer_point[0], now_inner_point[1]});
      } else {
        // 外
        vec.insert(vec.begin() + 1, std::vector<double>{now_outer_point[0],
                                                        next_inner_point[1]});
        vec.emplace_back(
            std::vector<double>{next_inner_point[0], now_outer_point[1]});
      }
    }
  }
}

// 按顺时针/逆时针传入
bool Mpcc::InRec(std::vector<std::vector<double>>& rec, double x, double y) {
  bool flag1 = ((rec[0][0] - x) * (rec[1][0] - rec[0][0]) +
                (rec[0][1] - y) * (rec[1][1] - rec[0][1])) < 0;
  bool flag2 = ((rec[1][0] - x) * (rec[2][0] - rec[1][0]) +
                (rec[1][1] - y) * (rec[2][1] - rec[1][1])) < 0;
  bool flag3 = ((rec[2][0] - x) * (rec[3][0] - rec[2][0]) +
                (rec[2][1] - y) * (rec[3][1] - rec[2][1])) < 0;
  bool flag4 = ((rec[3][0] - x) * (rec[0][0] - rec[3][0]) +
                (rec[3][1] - y) * (rec[0][1] - rec[3][1])) < 0;
  return flag1 & flag2 & flag3 & flag4;
}