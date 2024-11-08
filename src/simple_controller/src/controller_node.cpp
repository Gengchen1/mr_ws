#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "controller.h"

volatile bool odo_received = false;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;

void on_odo(const nav_msgs::msg::Odometry::SharedPtr /* odom */) {
  odo_received = true;
  odo_sub.reset();
}

void parameter_callback(const std::shared_ptr<rclcpp::Node> node) {
  // 将 node 强制转换为 Controller 类型
  auto controller_node = std::static_pointer_cast<simple_controller::Controller>(node);
  // 获取参数
  double proportional = controller_node->get_parameter("proportional").as_double();
  double differential = controller_node->get_parameter("differential").as_double();
  double integral = controller_node->get_parameter("integral").as_double();

  RCLCPP_INFO(node->get_logger(), "Reconfigure %f %f %f", proportional, differential, integral);

  // 更新控制器参数
  controller_node->reset(proportional, differential, integral);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // 创建 Controller 节点
  auto node = std::make_shared<simple_controller::Controller>("simple_controller");

  // 参数已在 Controller 构造函数中声明，无需再次声明

  // 订阅里程计数据，等待收到数据后再继续
  odo_sub = node->create_subscription<nav_msgs::msg::Odometry>("odom", 1, on_odo);
  while (rclcpp::ok() && !odo_received) {
    rclcpp::spin_some(node);
  }
  RCLCPP_DEBUG(node->get_logger(), "odo received");

  // 调用参数回调函数，传入节点
  parameter_callback(node);

  // 添加参数更新回调
  auto param_callback = [node](const std::vector<rclcpp::Parameter> &) -> rcl_interfaces::msg::SetParametersResult {
    parameter_callback(node);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  };
  auto callback_handle = node->add_on_set_parameters_callback(param_callback);

  // 开始运���节点
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

