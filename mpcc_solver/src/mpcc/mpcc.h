// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma onces

#include "config.h"
#include "types.h"
#include "Params/params.h"
#include "Spline/arc_length_spline.h"
#include "Model/model.h"
#include "Model/integrator.h"
#include "Cost/cost.h"
#include "Constraints/constraints.h"
#include "Constraints/bounds.h"

#include "Interfaces/solver_interface.h"
#include "Interfaces/hpipm_interface.h"

#include <array>
#include <memory>
#include <ros/ros.h>
#include <ratio>


struct OptVariables {
  State xk;
  Input uk;
};

struct Stage {
  LinModelMatrix lin_model;
  CostMatrix cost_mat;
  ConstrainsMatrix constrains_mat;

  Bounds_x u_bounds_x;
  Bounds_x l_bounds_x;

  Bounds_u u_bounds_u;
  Bounds_u l_bounds_u;

  Bounds_s u_bounds_s;
  Bounds_s l_bounds_s;

  //nx    -> number of states
  //nu    -> number of inputs
  //nbx   -> number of bounds on x
  //nbu   -> number of bounds on u
  //ng    -> number of polytopic constratins
  //ns   -> number of soft constraints
  int nx, nu, nbx, nbu, ng, ns;
};

struct MPCReturn {
  const Input u0;
  const std::array<OptVariables,N+1> mpc_horizon;
  const double time_total;
};

class MPC {
 public:
  MPCReturn runMPC(State &x0);

  MPC();
  MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts,const PathToJson &path);

private:
  bool valid_initial_guess_;

  std::array<Stage, N + 1> stages_;

  std::array<OptVariables, N + 1> initial_guess_;
  std::array<OptVariables, N + 1> optimal_solution_;

  void setMPCProblem();

  void setStage(const State &xk, const Input &uk, int time_step);

  CostMatrix normalizeCost(const CostMatrix &cost_mat);
  LinModelMatrix normalizeDynamics(const LinModelMatrix &lin_model);
  ConstrainsMatrix normalizeCon(const ConstrainsMatrix &con_mat);
  std::array<OptVariables,N+1> deNormalizeSolution(const std::array<OptVariables,N+1> &solution);

  void updateInitialGuess(const State &x0);

  void generateNewInitialGuess(const State &x0);

  void unwrapInitialGuess();

  std::array<OptVariables, N + 1> sqpSolutionUpdate(const std::array<OptVariables, N + 1> &last_solution,
                                                    const std::array<OptVariables, N + 1> &current_solution);

  int n_sqp_;
  double sqp_mixing_;
  int n_non_solves_;
  int n_no_solves_sqp_;
  int n_reset_;

  const double Ts_;

  Model model_;
  Integrator integrator_;
  Cost cost_;
  Constraints constraints_;

  Bounds bounds_;
  NormalizationParam normalization_param_;
  Param param_;

  std::unique_ptr<SolverInterface> solver_interface_;
};
