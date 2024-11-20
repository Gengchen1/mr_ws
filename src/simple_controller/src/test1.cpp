#include <rclcpp/rclcpp.hpp>

class TestNode : public rclcpp::Node {
 public:
  TestNode() : Node("test_node") {
    RCLCPP_DEBUG(rclcpp::get_logger("test_node"), "This is a DEBUG message.");
    RCLCPP_WARN(this->get_logger(), "This is an INFO message.");
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}