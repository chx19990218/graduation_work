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

  // Qk.resize(horizon * numState, horizon * numState);
  // qk.resize(horizon * numState, 1);
  // AA.resize(horizon * numState, numState);
  // BB.resize(horizon * numState, horizon * numInput);
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
  // // initialize AA & BB:
  // Eigen::SparseMatrix<double> tmpA = model.Ad;
  // Eigen::SparseMatrix<double> tmpB(numState, horizon * numInput);
  // sp::colMajor::setRows(AA, model.Ad, 0);
  // sp::colMajor::setBlock(BB, model.Bd, 0, 0);
  // for (int k = 1; k < horizon; ++k) {
  //   tmpA = model.Ad * tmpA;
  //   sp::colMajor::setRows(AA, tmpA, k * numState);
  //   tmpB = model.Ad *
  //          BB.block((k - 1) * numState, 0, numState, horizon * numInput);
  //   sp::colMajor::setRows(BB, tmpB, k * numState);
  //   sp::colMajor::setBlock(BB, model.Bd, k * numState, k * numInput);
  // }
  // AAT = AA.transpose();
  // BBT = BB.transpose();
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

void Mpcc::GetOptimalTheta(std::vector<double>& optimal_theta) {
  optimal_theta.clear();
  double v = 1.0;
  if (init_status) {
    for (int i = 0; i < horizon; i++) {
      optimal_theta.emplace_back(
          0.0 +
          Ts * v * (i + 1));  // TODO:确定无人机起始位置，然后确定起始theta
    }
  } else {
    for (int i = 1; i < horizon; i++) {
      optimal_theta.emplace_back(stage[i].state.back());
    }
    optimal_theta.emplace_back(stage.back().state.back() + stage.back().u.back() * Ts);
  }
}

void Mpcc::CalculateCost(const Resample& referenceline) {
  stage.clear();
  std::vector<double> optimal_theta;
  GetOptimalTheta(optimal_theta);
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

    double r_x = x - dx_dtheta * theta;
    double r_y = y - dy_dtheta * theta;
    double r_z = z - dz_dtheta * theta;
    Eigen::SparseMatrix<double> grad_x(state_dim_, 1);
    Eigen::SparseMatrix<double> grad_y(state_dim_, 1);
    Eigen::SparseMatrix<double> grad_z(state_dim_, 1);
    grad_x.coeffRef(0, 0) = 1;
    grad_y.coeffRef(2, 0) = 1;
    grad_z.coeffRef(4, 0) = 1;
    grad_x.coeffRef(6, 0) = -dx_dtheta;
    grad_y.coeffRef(6, 0) = -dy_dtheta;
    grad_z.coeffRef(6, 0) = -dz_dtheta;
    stage[i].Qn = grad_x * Eigen::SparseMatrix<double>(grad_x.transpose()) +
                  grad_y * Eigen::SparseMatrix<double>(grad_y.transpose()) +
                  grad_z * Eigen::SparseMatrix<double>(grad_z.transpose());
    stage[i].qn = -2 * r_x * grad_x - 2 * r_y * grad_y - 2 * r_z * grad_z;
  }

  // qn.coeffRef(numState - Model::numOrder + 1, 0) = -qVTheta;
  // sp::colMajor::setBlock(Qk, Qn, numState * horizon_, numState * horizon_);
  // sp::colMajor::setBlock(qk, qn, numState * horizon_, 0);
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
// int MpcSolver::solveMpcQp(Eigen::SparseMatrix<double>& stateTmp) {
//   Eigen::SparseMatrix<double> state = stateTmp;
  
//   Eigen::SparseMatrix<double> inputPredict1toN =
//       inputPredict.block(0, 1, numInput, horizon - 1);
//   sp::colMajor::setCols(inputPredict, inputPredict1toN, 1);
//   int solveStatus = 1;

//   Ck.resize(0, 0);
//   Ckb.resize(0, 0);

//   displayPtr->displayDrone(state);
//   displayPtr->displayTheta(state);
//   double theta = 0;
//   // displayPtr->clearTunnels();
//   for (int horizonI = 0; horizonI < horizon; ++horizonI) {
//     if (initStatus) {
//       theta = model.ts * horizon;
//     } else {
//       theta = state.coeffRef(numState - Model::numOrder, 0);
//     }
//     calculateCost(theta, horizonI);
//     if (corridorConstrains) {
//       calculateConstrains(state, horizonI);
//     }
//     state = model.Ad * state + model.Bd * inputPredict.col(horizonI);
//   }

//   Q = BBT * Qk * BB + In;
//   c = BBT * Qk * AA * stateTmp + BBT * qk;


//   C = BB;

//   // ! set terminal velocity constrain
//   Vector3d pos, vel;
//   map.getGlobalCommand(theta, pos, vel);
//   xu.coeffRef(numState * (horizon - 1) + 1, 0) = 1 * fabs(vel(0));
//   xu.coeffRef(numState * (horizon - 1) + 4, 0) = 1 * fabs(vel(1));
//   xu.coeffRef(numState * (horizon - 1) + 7, 0) = 1 * fabs(vel(2));
//   xl.coeffRef(numState * (horizon - 1) + 1, 0) = -1 * fabs(vel(0));
//   xl.coeffRef(numState * (horizon - 1) + 4, 0) = -1 * fabs(vel(1));
//   xl.coeffRef(numState * (horizon - 1) + 7, 0) = -1 * fabs(vel(2));

//   cupp = xu - SparseMatrix<double>(AA * stateTmp);
//   clow = xl - SparseMatrix<double>(AA * stateTmp);

//   if (Ck.rows() != 0) {
//     // lower bound should be quite small
//     Eigen::SparseMatrix<double> zeroRows(Ck.rows(), 1);
//     for (int i = 0; i < Ck.rows(); ++i) {
//       zeroRows.coeffRef(i, 0) = -1e10;
//     }
//     sp::colMajor::addRows(clow, zeroRows);
//     zeroRows = -Ck * AA * stateTmp - Ckb;
//     sp::colMajor::addRows(cupp, zeroRows);
//     SparseMatrix<double> Ck2 = Ck * BB;
//     sp::colMajor::addRows(C, Ck2);
//     sp::colMajor::addRows(C, Inu);
//     sp::colMajor::addRows(cupp, uu);
//     sp::colMajor::addRows(clow, ul);
//   }

//   clock_t t_osqp_start = clock();
//   osqpInterface.updateMatrices(Q, c, A, b, C, clow, cupp, ul, uu);
//   osqpInterface.solveQP();
//   solveStatus = osqpInterface.solveStatus();

//   auto solPtr = osqpInterface.solPtr();
//   clock_t t_osqp_over = clock();
//   double time = (double)(t_osqp_over - t_osqp_start) / CLOCKS_PER_SEC;

//   if (solveStatus == OSQP_SOLVED) {

//     state = stateTmp;
//     for (int i = 0; i < horizon; ++i) {
//       for (int j = 0; j < numInput; ++j) {
//         inputPredict.coeffRef(j, i) = solPtr->x[i * numInput + j];
//       }
//       state = model.Ad * state + model.Bd * inputPredict.col(i);
//       sp::colMajor::setCols(statePredict, state, i);
//     }
//   } else {
//     sp::colMajor::setCols(inputPredict, inputPredict1toN, 0);
//     SparseMatrix<double> statePredict1toN =
//         statePredict.block(0, 1, numState, horizon - 1);
//     sp::colMajor::setCols(statePredict, statePredict1toN, 0);
//   }

//   if (initStatus) {
//     initStatus = false;
//   }
//   displayPtr->displayPredict(statePredict);
//   if (solveStatus == OSQP_SOLVED)
//     return 0;  // 0 for solved
//   else
//     return 1;
// };
