#include <rclcpp/rclcpp.hpp>
#include "mpc_controller.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // 使用默认时钟
  auto node = std::make_shared<mpc_controller::MPCController>("mpc_controller");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

