#include "velocity_controller.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<velocity_controller::Controller>("velocity_controller");
  if (node->declare_parameter("/use_sim_time", false)) {
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      node->last_timer_time = node->now();
      if (node->last_timer_time.seconds() != 0.0) {
        break;
      }
    }
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
}
