#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "controller.h"

// volatile关键字声明的变量，编译器对访问该变量的代码就不再进行优化，
// 从而可以提供对特殊地址的稳定访问
volatile bool odo_received = false;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
void on_odo(const nav_msgs::msg::Odometry::SharedPtr /* odom */) {
  odo_received = true;
  odo_sub.reset();
}

// 可以通过虚函数来使基类调用子类的成员函数
void parameter_callback(const std::shared_ptr<rclcpp::Node> node,
simple_controller::Controller& ctrl) {
  // 创建一个用于从ros2服务器获取参数更新参数的实例用户
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  // 等待参数服务在1秒内变得可用, 如果不可用，则记录错误信息并终止函数执行。
  if (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node->get_logger(), "Parameter service not available");
    return;
  }
  auto parameters = parameters_client->get_parameters({"proportional", "differential", "integral"});
  double proportional = parameters[0].as_double();
  double differential = parameters[1].as_double();
  double integral = parameters[2].as_double();

  RCLCPP_INFO(node->get_logger(), "Reconfigure %f %f %f", proportional, differential, integral);
  ctrl.reset(proportional, differential, integral);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("simple_controller");

  node->declare_parameter("proportional", 0.0);
  node->declare_parameter("differential", 0.0);
  node->declare_parameter("integral", 0.0);
  
  // 在启动控制器之前尝试从机器人接收第一条消息
  // 这个东西是为gazebo引入的
  odo_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 1, on_odo);
  // ROS 2系统正常运行且还没有接收到里程计数据时，
  // 不断调用rclcpp::spin_some(node)来处理传入的消息和服务请求
  // 确保节点在等待里程计数据的同时，仍然能够处理其他的ROS 2消息和服务
  while (rclcpp::ok() && !odo_received) {
    // 只处理回调函数一次，且不会等待新消息到来
    rclcpp::spin_some(node);
  }
  RCLCPP_DEBUG(node->get_logger(), "odo received");
  
  simple_controller::Controller Cntrl;

  // 初始化参数
  parameter_callback(node, Cntrl);

  // 设置参数回调函数,lambda表达式 
  auto param_callback = [node, &Cntrl](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::SetParametersResult {
    parameter_callback(node, Cntrl);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  };
  node->add_on_set_parameters_callback(param_callback);
}

