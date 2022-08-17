// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <eigen3/Eigen/Dense>

#define NX 11
#define NU 4
#define NB 13 //max number of bounds
#define NPC 3 //number of polytopic constraints
#define NS 3
static constexpr int N = 60;
static constexpr double INF = 1E5;
static constexpr int N_SPLINE = 5000;

typedef Eigen::Matrix<double,NX,1> StateVector;
typedef Eigen::Matrix<double,NU,1> InputVector;

typedef Eigen::Matrix<double,NX,NX> A;
typedef Eigen::Matrix<double,NX,NU> B;
typedef Eigen::Matrix<double,NX,1> g;

typedef Eigen::Matrix<double,NX,NX> Q;
typedef Eigen::Matrix<double,NU,NU> R;
typedef Eigen::Matrix<double,NX,NU> S;

typedef Eigen::Matrix<double,NX,1> q;
typedef Eigen::Matrix<double,NU,1> r;

typedef Eigen::Matrix<double,NPC,NX> C;
typedef Eigen::Matrix<double,1,NX> C_i;
typedef Eigen::Matrix<double,NPC,NU> D;
typedef Eigen::Matrix<double,NPC,1> d;

typedef Eigen::Matrix<double,NS,NS> Z;
typedef Eigen::Matrix<double,NS,1> z;

typedef Eigen::Matrix<double,NX,NX> TX;
typedef Eigen::Matrix<double,NU,NU> TU;
typedef Eigen::Matrix<double,NS,NS> TS;

typedef Eigen::Matrix<double,NX,1> Bounds_x;
typedef Eigen::Matrix<double,NU,1> Bounds_u;
typedef Eigen::Matrix<double,NS,1> Bounds_s;

// Differential model
struct State {
  double x_;
  double y_;
  double z_;
  double vx_;
  double vy_;
  double vz_;
  double theta_;
  double ax_;
  double ay_;
  double az_;
  double v_theta_;

  void Reset() {
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
    vz_ = 0.0;
    theta_ = 0.0;
    ax_ = 0.0;
    ay_ = 0.0;
    az_ = 0.0;
    v_theta_ = 0.0;
  }
  void Unwrap(double track_length) {
    while (theta_ > track_length) {
      theta_ -= track_length;
    } 
    while (theta_ < 0) {
      theta_ += track_length;
    }
  }
};

struct Input {
  double d_ax_;
  double d_ay_;
  double d_az_;
  double d_v_theta_;

  void Reset() {
    d_ax_ = 0.0;
    d_ay_ = 0.0;
    d_az_ = 0.0;
    d_v_theta_ = 0.0;
  }
};

// StateVector StateToVector(const State &state) {
//     StateVector temp;
//     temp(0) = state.x_;
//     temp(1) = state.y_;
//     temp(2) = state.z_;
//     temp(3) = state.vx_;
//     temp(4) = state.vy_;
//     temp(5) = state.vz_;
//     temp(6) = state.theta_;
//     temp(7) = state.ax_;
//     temp(8) = state.ay_;
//     temp(9) = state.az_;
//     temp(10) = state.v_theta_;
//     return temp;
// }
// InputVector InputToVector(const Input &input) {
//   InputVector temp = {input.d_ax_, input.d_ay_, input.d_az_, input.d_v_theta_};
//   return temp;
// }
// State VectorToState(const StateVector &state_vector) {
//     State temp;
//     temp.x_ = state_vector(0);
//     temp.y_ = state_vector(1);
//     temp.z_ = state_vector(2);
//     temp.vx_ = state_vector(3);
//     temp.vy_ = state_vector(4);
//     temp.vz_ = state_vector(5);
//     temp.theta_ = state_vector(6);
//     temp.ax_ = state_vector(7);
//     temp.ay_ = state_vector(8);
//     temp.az_ = state_vector(9);
//     temp.v_theta_ = state_vector(10);
//     return temp;
// }
// Input VectorToInput(const InputVector &input_vector) {
//     Input temp;
//     temp.d_ax_ = input_vector(0);
//     temp.d_ay_ = input_vector(1);
//     temp.d_az_ = input_vector(2);
//     temp.d_v_theta_ = input_vector(3);
//     return temp;
// }