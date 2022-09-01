// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "mpc.h"

MPC::MPC() : Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

MPC::MPC(int n_sqp, int n_reset,double sqp_mixing, double Ts,const PathToJson &path)
    :Ts_(Ts),
    valid_initial_guess_(false),
    solver_interface_(new HpipmInterface()),
    param_(Param(path.param_path)),
    normalization_param_(NormalizationParam(path.normalization_path)),
    bounds_(BoundsParam(path.bounds_path)),
    constraints_(Constraints(Ts,path)),
    cost_(Cost(path)),
    integrator_(Integrator(Ts,path)),
    model_(Model(Ts,path)),
    track_(ArcLengthSpline(path)) {
    n_sqp_ = n_sqp;
    sqp_mixing_ = sqp_mixing;
    n_non_solves_ = 0;
    n_no_solves_sqp_ = 0;
    n_reset_ = n_reset;
}

void MPC::setMPCProblem() {
  for (int i = 0; i <= N; i++) {
    setStage(initial_guess_[i].xk, initial_guess_[i].uk, i);
  }
}

void MPC::setStage(const State &xk, const Input &uk, const int time_step) {
  stages_[time_step].nx = NX;
  stages_[time_step].nu = NU;

  if(time_step == 0) {
    stages_[time_step].ng = 0;
    stages_[time_step].ns = 0;
  } else {
    stages_[time_step].ng = NPC;
    stages_[time_step].ns = NS;
  }

  State xk_nz = xk;
  xk_nz.vxNonZero(param_.vx_zero);

  stages_[time_step].cost_mat = normalizeCost(cost_.getCost(track_,xk_nz,time_step));
  stages_[time_step].lin_model = normalizeDynamics(model_.getLinModel(xk_nz,uk));
  stages_[time_step].constrains_mat = normalizeCon(constraints_.getConstraints(track_,xk_nz,uk));

  stages_[time_step].l_bounds_x = normalization_param_.T_x_inv * bounds_.getBoundsLX();
  stages_[time_step].u_bounds_x = normalization_param_.T_x_inv * bounds_.getBoundsUX();
  stages_[time_step].l_bounds_u = normalization_param_.T_u_inv * bounds_.getBoundsLU();
  stages_[time_step].u_bounds_u = normalization_param_.T_u_inv * bounds_.getBoundsUU();
  stages_[time_step].l_bounds_s = normalization_param_.T_s_inv * bounds_.getBoundsLS();
  stages_[time_step].u_bounds_s = normalization_param_.T_s_inv * bounds_.getBoundsUS();

  stages_[time_step].l_bounds_x(si_index.s) = normalization_param_.T_x_inv(si_index.s,si_index.s)*
                                              (initial_guess_[time_step].xk.s - param_.s_trust_region);//*initial_guess_[time_step].xk.vs;
  stages_[time_step].u_bounds_x(si_index.s) = normalization_param_.T_x_inv(si_index.s,si_index.s)*
                                              (initial_guess_[time_step].xk.s + param_.s_trust_region);//*initial_guess_[time_step].xk.vs;
}

CostMatrix MPC::normalizeCost(const CostMatrix &cost_mat) {
  const Q_MPC Q = normalization_param_.T_x*cost_mat.Q*normalization_param_.T_x;
  const R_MPC R = normalization_param_.T_u*cost_mat.R*normalization_param_.T_u;
  const q_MPC q = normalization_param_.T_x*cost_mat.q;
  const r_MPC r = normalization_param_.T_u*cost_mat.r;
  const Z_MPC Z = normalization_param_.T_s*cost_mat.Z*normalization_param_.T_s;
  const z_MPC z = normalization_param_.T_s*cost_mat.z;
  return {Q,R,S_MPC::Zero(),q,r,Z,z};
}

LinModelMatrix MPC::normalizeDynamics(const LinModelMatrix &lin_model) {
    const A_MPC A = normalization_param_.T_x_inv*lin_model.A*normalization_param_.T_x;
    const B_MPC B = normalization_param_.T_x_inv*lin_model.B*normalization_param_.T_u;
    const g_MPC g = normalization_param_.T_x_inv*lin_model.g;
    return {A,B,g};
}

ConstrainsMatrix MPC::normalizeCon(const ConstrainsMatrix &con_mat) {
  const C_MPC C = con_mat.C*normalization_param_.T_x;
  const D_MPC D =  con_mat.D*normalization_param_.T_u;
  const d_MPC dl = con_mat.dl;
  const d_MPC du = con_mat.du;
  return {C,D,dl,du};
}

std::array<OptVariables,N+1> MPC::deNormalizeSolution(const std::array<OptVariables,N+1> &solution) {
  std::array<OptVariables, N + 1> denormalized_solution;
  StateVector updated_x_vec;
  InputVector updated_u_vec;
  for (int i = 0; i <= N; i++) {
      updated_x_vec = normalization_param_.T_x*stateToVector(solution[i].xk);
      updated_u_vec = normalization_param_.T_u*inputToVector(solution[i].uk);

      denormalized_solution[i].xk = vectorToState(updated_x_vec);
      denormalized_solution[i].uk = vectorToInput(updated_u_vec);
  }
  return denormalized_solution;
}


void MPC::updateInitialGuess(const State &x0) {
  for (int i=1;i<N;i++) {
    initial_guess_[i-1] = initial_guess_[i];
  }
  initial_guess_[0].xk = x0;
  initial_guess_[0].uk.setZero();

  initial_guess_[N-1].xk = initial_guess_[N-2].xk;
  initial_guess_[N-1].uk.setZero();// = initial_guess_[N-2].uk;

  initial_guess_[N].xk = integrator_.RK4(initial_guess_[N-1].xk,initial_guess_[N-1].uk,Ts_);
  initial_guess_[N].uk.setZero();

  unwrapInitialGuess();
}

// alternatively OptVariables MPC::unwrapInitialGuess(const OptVariables &initial_guess)
void MPC::unwrapInitialGuess() {
  double L = track_.getLength();
  for (int i=1; i<=N; i++) {
    if ((initial_guess_[i].xk.phi - initial_guess_[i-1].xk.phi) < -M_PI) {
      initial_guess_[i].xk.phi += 2.*M_PI;
    }
    if ((initial_guess_[i].xk.phi - initial_guess_[i-1].xk.phi) > M_PI) {
      initial_guess_[i].xk.phi -= 2.*M_PI;
    }
    if ((initial_guess_[i].xk.s - initial_guess_[i-1].xk.s) > L/2.) {
      initial_guess_[i].xk.s -= L;
    }
  }

}


MPCReturn MPC::runMPC(State &x0)
{
  auto t1 = ros::Time::now();
  int solver_status = -1;

  x0.s = track_.porjectOnSpline(x0);
  x0.unwrap(track_.getLength());

  updateInitialGuess(x0);

  setMPCProblem();
  State x0_normalized = vectorToState(normalization_param_.T_x_inv*stateToVector(x0));
  optimal_solution_ = solver_interface_->solveMPC(stages_, x0_normalized, &solver_status);
  optimal_solution_ = deNormalizeSolution(optimal_solution_);

  auto t2 = ros::Time::now();
  double time_span = (t2 - t1).toSec();

  return {initial_guess_[0].uk, initial_guess_, time_span};
}