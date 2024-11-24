#include "mpc.h"
#include "rclcpp/rclcpp.hpp"

USING_NAMESPACE_ACADO
namespace mpc_controller {

MPC::MPC(double steps, double dt, double max_vel, double max_acc, double max_delta,
         double max_delta_rate, double L, double kcte, double kepsi, double kv,
         double ksteer_cost)
    :
      t_end(steps * dt),
      steps(steps),  
      max_vel(max_vel),
      max_acc(max_acc),
      max_delta(max_delta),
      max_delta_rate(max_delta_rate),
      L(L),
      kcte(kcte),
      kepsi(kepsi),
      kev(kv),
      ksteer_cost(ksteer_cost) {
  // discrete time system
  f << dot(x) == vel * cos(fi);
  f << dot(y) == vel * sin(fi);
  f << dot(fi) == vel * tan(delta) / L;
  f << dot(delta) == delta_rate;
  f << dot(vel) == acc;
}

// 修改 solve 函数的参数，使其与头文件中的声明一致
void MPC::solve(double current_linear_velocity, double cmd_steer_angle, std::vector<double>& control_coefs,
               double& rate_u, double& acc_u, std::vector<double>& mpc_x, std::vector<double>& mpc_y) {
    RCLCPP_DEBUG(rclcpp::get_logger("MPC"), "solve mpc for v0 = %f angle0 = %f", current_linear_velocity, cmd_steer_angle);
    // RCLCPP_INFO(rclcpp::get_logger("MPC"), "delta0: %f, max_delta: %f", cmd_steer_angle, max_delta);
    assert(std::abs(cmd_steer_angle) < max_delta);
    ACADO::OCP ocp(t_start, t_end, steps);
    // constrains
    ocp.subjectTo(f);
    ocp.subjectTo(AT_START, x == 0);
    ocp.subjectTo(AT_START, y == 0);
    ocp.subjectTo(AT_START, fi == 0);
    ocp.subjectTo(-max_acc <= acc <= max_acc);
    ocp.subjectTo(-max_delta_rate <= delta_rate <= max_delta_rate);
    ocp.subjectTo(-max_delta <= delta <= max_delta);
    ocp.subjectTo(AT_START, vel == current_linear_velocity);
    ocp.subjectTo(AT_START, delta == cmd_steer_angle);

    // 使用 control_coefs 替代 traj_coef
    assert(control_coefs.size() == 4);
    double& a0 = control_coefs[0];
    double& a1 = control_coefs[1];
    double& a2 = control_coefs[2];
    double& a3 = control_coefs[3];

    // minimization objectives
    Expression cte = pow(y - a0 - a1 * x - a2 * x * x - a3 * x * x * x, 2);
    Expression epsi = pow(fi - atan(a1 + 2 * a2 * x + 3 * a3 * x * x), 2);
    Expression verr = pow(vel - max_vel, 2);
    Expression steer_cost = pow(delta_rate, 2);
    //  double emax = std::max(0.25, a0*1.2);
    //  ocp.subjectTo(cte <= emax);

    ocp.minimizeMayerTerm(kcte * cte + kepsi * epsi + kev * verr +
                          ksteer_cost * steer_cost);
    //  ocp.subjectTo(AT_END, cte <= 0.1);
    //  ocp.subjectTo(AT_END, epsi <= 0.01);

    OptimizationAlgorithm alg(ocp);
    alg.set(PRINTLEVEL, NONE);
    RCLCPP_INFO(rclcpp::get_logger("MPC"), "start solving mpc");
    alg.solve();
    RCLCPP_INFO(rclcpp::get_logger("MPC"), "finished solving mpc");
    VariablesGrid controls;
    alg.getControls(controls);
    //  controls.print();
    VariablesGrid states;
    alg.getDifferentialStates(states);
    // states.print();

    rate_u = controls(0, 0);
    acc_u = controls(0, 1);
    RCLCPP_INFO(rclcpp::get_logger("MPC"), "delta = %f acc = %f", rate_u, acc_u);

    mpc_x.resize(states.getNumPoints());
    mpc_y.resize(mpc_x.size());
    for (std::size_t i = 0; i < mpc_x.size(); ++i) {
        mpc_x[i] = states(i, 0);
        mpc_y[i] = states(i, 1);
    }
}

// 定义析构函数
MPC::~MPC() {
  // 如果有需要清理的资源，可以在这里处理
}

} /* namespace mpc_controller */
