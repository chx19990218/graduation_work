// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "mpcc.h"

Mpcc::Mpcc() {
  // std::string path =
  //     ros::package::getPath("cmpcc") + "/config/mpcParameters.yaml";
  // YAML::Node node = YAML::LoadFile(path);
  // horizon = node["horizon"].as<int>();
  // corridorConstrains = node["corridorConstrains"].as<int>();
  // qVTheta = node["qVTheta"].as<double>();
  // stateUpper = node["stateUpper"].as<vector<double>>();
  // stateLower = node["stateLower"].as<vector<double>>();
  // inputUpper = node["inputUpper"].as<vector<double>>();
  // inputLower = node["inputLower"].as<vector<double>>();
  // displayPtr = new DisplayMsgs(map, horizon);
  // initStatus = true;
  stage.resize(horizon);
  stage[0].state[0] = 0.5;
  stage[0].state[1] = 0.0;
  stage[0].state[2] = 1.18;
  stage[0].state[3] = 0.0;
  stage[0].state[4] = 1.0;
  stage[0].state[5] = 0.0;
  stage[0].state[6] = 0.0;

  Q.resize(horizon * state_dim_, horizon * state_dim_);
  q.resize(horizon * state_dim_, 1);
  AA.resize(horizon * state_dim_, state_dim_);
  BB.resize(horizon * state_dim_, horizon * control_dim_);

  Ad.resize(state_dim_, state_dim_);
  std::vector<Eigen::Triplet<double>> triplets;
  for (int i = 0; i < state_dim_ - 1; i++) {
    triplets.push_back(Eigen::Triplet<double>(i, i, 1));
  }
  triplets.push_back(Eigen::Triplet<double>(0, 1, Ts));
  triplets.push_back(Eigen::Triplet<double>(2, 3, Ts));
  triplets.push_back(Eigen::Triplet<double>(4, 5, Ts));
  Ad.setFromTriplets(triplets.begin(), triplets.end());

  Bd.resize(state_dim_, control_dim_);
  triplets.clear();
  triplets.push_back(Eigen::Triplet<double>(1, 0, Ts));
  triplets.push_back(Eigen::Triplet<double>(3, 1, Ts));
  triplets.push_back(Eigen::Triplet<double>(5, 2, Ts));
  triplets.push_back(Eigen::Triplet<double>(6, 3, Ts));
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
  xl.resize(horizon * state_dim_, 1);
  xu.resize(horizon * state_dim_, 1);
  Eigen::SparseMatrix<double> stateUs(state_dim_, 1);
  Eigen::SparseMatrix<double> stateLs(state_dim_, 1);
  for (int i = 0; i < state_dim_; i++) {
    stateUs.coeffRef(i, 0) = 100.0;
    stateLs.coeffRef(i, 0) = -100.0;
  }
  for (int i = 0; i < horizon; i++) {
    sp::colMajor::setRows(xl, stateLs, state_dim_ * i);
    sp::colMajor::setRows(xu, stateUs, state_dim_ * i);
  }
  inputPredict.resize(control_dim_, horizon);
  statePredict.resize(state_dim_, horizon);
  ul.resize(horizon * control_dim_, 1);
  uu.resize(horizon * control_dim_, 1);
  A.resize(0, 0);
  b.resize(0, 0);

  // In.resize(horizon * numInput, horizon * numInput);
  // A.resize(0, 0);
  // b.resize(0, 0);
  // xl.resize(horizon * numState, 1);
  // xu.resize(horizon * numState, 1);
  // ul.resize(horizon * numInput, 1);
  // uu.resize(horizon * numInput, 1);
  // inputPredict.resize(numInput, horizon);
  // statePredict.resize(numState, horizon);
  // // initialize xl,xu,ul,uu:
  // Eigen::SparseMatrix<double> stateUs(numState, 1);
  // Eigen::SparseMatrix<double> stateLs(numState, 1);
  // Eigen::SparseMatrix<double> inputUs(numInput, 1);
  // Eigen::SparseMatrix<double> inputLs(numInput, 1);
  // for (int i = 0; i < numState; ++i) {
  //   stateUs.coeffRef(i, 0) = stateUpper[i];
  //   stateLs.coeffRef(i, 0) = stateLower[i];
  // }
  // for (int i = 0; i < numInput; ++i) {
  //   inputUs.coeffRef(i, 0) = inputUpper[i];
  //   inputLs.coeffRef(i, 0) = inputLower[i];
  // }
  // for (int i = 0; i < horizon; ++i) {
  //   sp::colMajor::setRows(xl, stateLs, numState * i);
  //   sp::colMajor::setRows(xu, stateUs, numState * i);
  //   sp::colMajor::setRows(ul, inputLs, numInput * i);
  //   sp::colMajor::setRows(uu, inputUs, numInput * i);
  //   xu.coeffRef(i * numState + 9, 0) = map.thetaMax;
  // }

  // // initialize In: -> penalty minium snap
  // for (int i = 0; i < horizon * numInput; ++i) {
  //   In.coeffRef(i, i) = 1e-4;
  //   if (i < (horizon - 1) * numInput) {
  //     In.coeffRef(i, i + numInput) = -1e-6;
  //     In.coeffRef(i + numInput, i) = -1e-6;
  //   }
  // }
  // // initialize Inu:
  // Inu.resize(horizon * numInput, horizon * numInput);
  // Inu.setIdentity();
}

void Mpcc::GetOptimalTheta() {
  auto temp = optimal_theta;
  optimal_theta.clear();
  double v = 0.1;
  if (init_status) {
    for (int i = 0; i < horizon; i++) {
      optimal_theta.emplace_back(
          0.0 +
          Ts * v * (i + 1));  // TODO:确定无人机起始位置，然后确定起始theta
    }
  } else {
    for (int i = 1; i < horizon; i++) {
      optimal_theta.emplace_back(std::fmod(temp[i], max_theta_));
    }
    optimal_theta.emplace_back(std::fmod(
        temp.back() + inputPredict.coeffRef(control_dim_ - 1, horizon - 1) * Ts,
        max_theta_));
  }
}

void Mpcc::CalculateCost(const Resample& referenceline,
                         Eigen::SparseMatrix<double>& x0) {
  max_theta_ = referenceline.spline.getLength();
  stage.clear();
  GetOptimalTheta();
  for (int i = 0; i < horizon; i++) {
    double theta = optimal_theta[i];  // TODO 考虑闭环，theta跑一圈
    auto pos_xy = referenceline.spline.getPostion(theta);
    auto vel_xy = referenceline.spline.getDerivative(theta);
    double x = pos_xy[0];
    double y = pos_xy[1];
    double z = 1.0;
    double dx_dtheta = vel_xy[0];
    double dy_dtheta = vel_xy[1];
    double dz_dtheta = 0.0;
    // std::cout<<theta<<","<<x<<","<<y<<","<<dx_dtheta<<","<<dy_dtheta<<std::endl;

    double r_x = x - dx_dtheta * theta;
    double r_y = y - dy_dtheta * theta;
    double r_z = z - dz_dtheta * theta;
    Eigen::SparseMatrix<double> grad_x(state_dim_, 1);
    Eigen::SparseMatrix<double> grad_y(state_dim_, 1);
    Eigen::SparseMatrix<double> grad_z(state_dim_, 1);
    grad_x.coeffRef(0, 0) = 1.0;
    grad_y.coeffRef(2, 0) = 1.0;
    grad_z.coeffRef(4, 0) = 1.0;
    grad_x.coeffRef(6, 0) = -dx_dtheta;
    grad_y.coeffRef(6, 0) = -dy_dtheta;
    grad_z.coeffRef(6, 0) = -dz_dtheta;
    Eigen::SparseMatrix<double> Qn =
        grad_x * Eigen::SparseMatrix<double>(grad_x.transpose()) +
        grad_y * Eigen::SparseMatrix<double>(grad_y.transpose()) +
        grad_z * Eigen::SparseMatrix<double>(grad_z.transpose());
    Eigen::SparseMatrix<double> qn =
        -2 * r_x * grad_x - 2 * r_y * grad_y - 2 * r_z * grad_z;
    sp::colMajor::setBlock(Q, Qn, state_dim_ * i, state_dim_ * i);
    sp::colMajor::setBlock(q, qn, state_dim_ * i, 0);
  }

  Eigen::SparseMatrix<double> progress(control_dim_ * horizon, 1);
  double gamma = 0.0001;
  for (int i = 0; i < horizon; i++) {
    progress.coeffRef(4 * i + 3, 0) = gamma;
  }

  H = 2 * BBT * Q * BB;
  // std::cout<<AA<<std::endl;
  f = 2 * BBT * Q * AA * x0 + BBT * q - progress;
}

// void MpcSolver::calculateConstrains(Eigen::SparseMatrix<double>& stateTmp,
//                                     int horizon_) {
//   assert(stateTmp.rows() == numState);
//   double theta_ = stateTmp.coeffRef(numState - Model::numOrder, 0);
//   double theta = std::fmod(theta_ + map.thetaMax, map.thetaMax);
//   // double theta = theta_>map.thetaMax ? map.thetaMax:theta_;
//   int numConstrains = 0;
//   Polyhedron chosenPoly;
//   if (theta < 1e-6) {
//     theta = 1e-6;
//   }
//   Eigen::Vector3d position, tangentLine;
//   map.getGlobalCommand(theta, position, tangentLine);

//   // find the max Polyhedron intersecting theta face
//   double maxArea = 0;
//   // cout << "theta: " << stateTmp.coeffRef(7,0) << endl;
//   // find tunnel for this horizon:
//   for (int i = 0; i < map.corridor.polys.size(); ++i) {
//     bool inThisPoly = false;
//     for (int k = 0; k < map.corridor.polys[i].starter.size(); ++k) {
//       bool in = theta >= map.corridor.polys[i].starter[k] &&
//                 theta <= map.corridor.polys[i].ender[k];
//       inThisPoly = inThisPoly || in;
//     }
//     if (inThisPoly) {
//       // std::cout << "polyindex: " << i << std::endl;
//       // cout << "pos: " << position.transpose() << endl;
//       // cout << "tan: " << tangentLine.transpose() << endl;
//       // cout << "i: " << i << endl;
//       map.corridor.FindPolygon(position, tangentLine, i);
//       if (maxArea < map.corridor.tunnelArea) {
//         maxArea = map.corridor.tunnelArea;
//         chosenPoly = map.corridor.tunnel;
//       }
//       // std::cout << "area: " << maxArea << endl;
//     }
//   }
//   displayPtr->displayOneTunnel(horizon_, theta);
//   numConstrains = chosenPoly.rows();

//   Cn.resize(numConstrains, numState);
//   Cnb.resize(numConstrains, 1);
//   for (int i = 0; i < numConstrains; ++i) {
//     for (int j = 0; j < Model::numDimention; ++j) {
//       Cn.coeffRef(i, Model::numOrder * j) = chosenPoly(i, j);
//     }
//     Cnb.coeffRef(i, 0) = chosenPoly(i, 3);
//   }
//   sp::colMajor::addBlock(Ck, Cn);
//   sp::colMajor::addRows(Ckb, Cnb);
// }
void Mpcc::SolveQp(Eigen::SparseMatrix<double>& stateTmp) {
  Eigen::SparseMatrix<double> state = stateTmp;

  C = BB;

  cupp = xu - Eigen::SparseMatrix<double>(AA * stateTmp);
  clow = xl - Eigen::SparseMatrix<double>(AA * stateTmp);

  // std::cout<<clow<<std::endl;
  osqpInterface.updateMatrices(H, f, A, b, C, clow, cupp, ul, uu);
  osqpInterface.solveQP();
  auto solveStatus = osqpInterface.solveStatus();

  auto solPtr = osqpInterface.solPtr();

  if (solveStatus == OSQP_SOLVED) {
    optimal_theta.clear();
    Eigen::SparseMatrix<double> state = stateTmp;

    stage.clear();
    for (int i = 0; i < horizon; i++) {
      for (int j = 0; j < control_dim_; j++) {
        inputPredict.coeffRef(j, i) = solPtr->x[i * control_dim_ + j];
        // stage[i].u[j] = solPtr->x[i*control_dim_+j];
      }
      state = Ad * state + Bd * inputPredict.col(i);
      sp::colMajor::setCols(statePredict, state, i);
      // for (int k = 0; k < state_dim_; k++) {
      //   stage[i].state[i] = state.coeffRef(k, 0);
      // }
      optimal_theta.emplace_back(state.coeffRef(6, 0));
    }
  } else {
    std::cout << "no solution" << std::endl;
  }

  if (init_status) {
    init_status = false;
  }
}