#ifndef MPC_H_
#define MPC_H_
#include <acado/acado_optimal_control.hpp>
#include <vector>

namespace mpc_controller
{

class MPC
{
public:
  // 确保构造函数参数数量为11个
  MPC(int steps, double dt, double max_vel, double max_acc, double max_delta, double max_delta_rate, double L,
      double kcte = 1.0, double kepsi = 1.0, double kev = 1.0, double ksteer_cost = 1.0);

  // 确保 getCoefsSize 方法存在，如果需要
  size_t getCoefsSize() const {
    return control_coefs.size();
  }

  // 修改 solve 方法的参数，使其与定义一致
  void solve(double current_linear_velocity, double cmd_steer_angle, std::vector<double>& control_coefs,
             double& param1, double& param2, std::vector<double>& mpc_x, std::vector<double>& mpc_y);

  // 声明析构函数
  virtual ~MPC();

private:
  std::vector<double> control_coefs;
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
};

} /* namespace mpc_controller */

#endif /* SRC_MPC_CONTROLLER_INCLUDE_MPC_H_ */
