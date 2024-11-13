#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "controller.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // 创建 Controller 节点
  auto node = std::make_shared<simple_controller::Controller>("simple_controller");

  // 移除以下重复的参数声明
  /*
  node->declare_parameter<double>("proportional", 1.0);
  node->declare_parameter<double>("differential", 0.0);
  node->declare_parameter<double>("integral", 0.0);
  */

  // 获取参数并设置控制器
  double proportional = node->get_parameter("proportional").as_double();
  double differential = node->get_parameter("differential").as_double();
  double integral = node->get_parameter("integral").as_double();
  node->reset(proportional, differential, integral);

  // 设置参数回调
  node->add_on_set_parameters_callback(
    [node](const std::vector<rclcpp::Parameter> &parameters) {
      for (const auto &param : parameters) {
        if (param.get_name() == "proportional" || param.get_name() == "differential" || param.get_name() == "integral") {
          double proportional = node->get_parameter("proportional").as_double();
          double differential = node->get_parameter("differential").as_double();
          double integral = node->get_parameter("integral").as_double();
          RCLCPP_INFO(node->get_logger(), "Reconfigure %f %f %f", proportional, differential, integral);
          node->reset(proportional, differential, integral);
          break;
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  // 订阅里程计数据，等待收到数据后再继续
  volatile bool odo_received = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
  odo_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 1,
    [&odo_received, &odo_sub](const nav_msgs::msg::Odometry::SharedPtr /* odom */) {
      odo_received = true;
      odo_sub.reset();
    });

  // while (rclcpp::ok() && !odo_received) {
  //   rclcpp::spin_some(node);
  // }
  // RCLCPP_DEBUG(node->get_logger(), "odo received");

  // 运行节点
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

