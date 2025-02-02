#pragma once

#include <acado/acado_optimal_control.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mpc_controller {
class MPC {
  ACADO::DifferentialState x, y, fi, delta, vel;
  ACADO::Control delta_rate, acc;
  ACADO::DifferentialEquation f;
  double t_start = 0;
  double t_end;
  double steps;

  double max_vel;
  double max_acc;
  double max_delta;
  double max_delta_rate;
  double L;
  double kcte;
  double kepsi;
  double kev;
  double ksteer_cost;

 public:
  MPC();

  virtual ~MPC();

  void init(int steps, double dt, double max_vel, double max_acc,
            double max_delta, double max_delta_rate, double L,
            double kcte = 1.0, double kepsi = 1.0, double kev = 1.0,
            double ksteer_cost = 1.0);

  void solve(double v0, double delta0, std::vector<double> &traj_coef,
             double &rate_u, double &acc_u, std::vector<double> &x,
             std::vector<double> &y);
};
}  // namespace mpc_controller