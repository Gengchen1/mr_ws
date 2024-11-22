#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <algorithm>
#include <chrono>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace velocity_controller {
class Controller : public rclcpp::Node {
 public:
  void on_command_velocity(const std_msgs::msg::Float32::SharedPtr msg);
  void on_odo(const nav_msgs::msg::Odometry::SharedPtr odom);
  void on_timer();
  rclcpp::Time last_timer_time;

  Controller(const std::string &name);
  ~Controller(); // 声明析构函数

 protected:
  double desired_velocity;
  double current_velocity;

  // 刹车发布
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr err_pub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub;
};
class PIDController {
 private:
  double kp_;
  double ki_;
  double kd_;
  double prev_error_;
  double integral_; 

 public:
  PIDController(double kp, double ki, double kd);
  double calculate(double error, double dt);
};

double PIDController::calculate(double error, double dt) {
  // rclcpp::Time time = this->now();
  // auto dt = (time - last_timer_time).seconds();
  // last_timer_time = time;
  double derivate = error - prev_error_ / dt;
  integral_ += error * dt;
  prev_error_ = error;
  return kp_ * error + ki_ * integral_ + kd_ * derivate;
}
PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

void Controller::on_command_velocity(
    const std_msgs::msg::Float32::SharedPtr msg) {
  desired_velocity = msg->data;
  std_msgs::msg::Float32 err;
  err.data = desired_velocity - current_velocity;
  err_pub->publish(err);
}

void Controller::on_odo(const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_velocity = odom->twist.twist.linear.x;
}

void Controller::on_timer() {
  rclcpp::Time time = this->now();
  auto dt = (time - last_timer_time).seconds();
  last_timer_time = time;
  std_msgs::msg::Float32 throttle_cmd;

  PIDController pid_linear(50, 0, 10);

  double error = desired_velocity - current_velocity;
  double control_signal = pid_linear.calculate(error, dt);

  double stauration_limit = 10.0;
  control_signal =
      std::min(std::max(control_signal, -stauration_limit), stauration_limit);

  if (control_signal > 0) {
    throttle_cmd.data = control_signal;
  } else {
    throttle_cmd.data = -1.0;
  }

  throttle_pub->publish(throttle_cmd);
}

Controller::Controller(const std::string &name)
    : Node(name),
      last_timer_time(this->now()),
      desired_velocity(0.0),
      current_velocity(0.0),
      throttle_pub(
          this->create_publisher<std_msgs::msg::Float32>("~/throttle", 1)),
      err_pub(
          this->create_publisher<std_msgs::msg::Float32>("~/velocity_err", 1)),
      timer(this->create_wall_timer(std::chrono::duration<double>(0.1),
                                    std::bind(&Controller::on_timer, this))),
      odo_sub(this->create_subscription<nav_msgs::msg::Odometry>(
          "~/odom", 1,
          std::bind(&Controller::on_odo, this, std::placeholders::_1))),
      cmd_sub(this->create_subscription<std_msgs::msg::Float32>(
          "~/velocity", 1,
          std::bind(&Controller::on_command_velocity, this,
                    std::placeholders::_1))) {}
Controller::~Controller() {}
}  // namespace velocity_controller

#endif