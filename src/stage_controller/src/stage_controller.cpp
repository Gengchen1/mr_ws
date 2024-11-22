#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

double car_length = 1.5;
double steering = 0;
double cmd_steering = 0.0;
double max_steering_rate = 1.0;
double max_steering = 0.5;
double velocity = 0;
double desired_velocity = 0;
double max_acc = 1.0;
double max_velocity = 10;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
rclcpp::Time last_timer_time;

double clamp(double cmd, double max_value) {
  return copysign(std::min(std::abs(cmd), max_value), cmd);
}

double clamp(double value, double cmd, double max_value, double max_rate, double dt) {
  cmd = clamp(cmd, max_value);
  auto diff = (cmd - value);
  diff = clamp(diff, max_rate * dt);
  return value + diff;
}

void on_steering(const std_msgs::msg::Float32::SharedPtr msg) {
  cmd_steering = msg->data;
}

void on_command_velocity(const std_msgs::msg::Float32::SharedPtr msg) {
  desired_velocity = msg->data;
}

void on_timer(const rclcpp::TimerBase::SharedPtr /*timer*/, rclcpp::Node::SharedPtr node) {
  auto t = node->now();
  auto dt = (t - last_timer_time).seconds();
  last_timer_time = t;
  
  geometry_msgs::msg::Twist cmd;
  velocity = clamp(velocity, desired_velocity, max_velocity, max_acc, dt);
  steering = clamp(steering, cmd_steering, max_steering, max_steering_rate, dt);
  
  cmd.linear.x = velocity;
  cmd.angular.z = velocity * tan(steering) / car_length;
  twist_pub->publish(cmd);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("stage_controller");

  car_length = node->declare_parameter("length", 1.5);
  RCLCPP_INFO(node->get_logger(), "Car length: %f", car_length);
  
  max_steering = node->declare_parameter("max_steering", 0.5);
  max_steering_rate = node->declare_parameter("max_steering_rate", 1.0);
  max_acc = node->declare_parameter("max_acc", 1.5);
  max_velocity = node->declare_parameter("max_velocity", 15.0);

  twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  auto steer_sub = node->create_subscription<std_msgs::msg::Float32>(
      "~/steering", 1, on_steering);
  auto vel_sub = node->create_subscription<std_msgs::msg::Float32>(
      "~/velocity", 1, on_command_velocity);

  last_timer_time = node->now();
  auto timer_callback = [node]() { on_timer(nullptr, node); };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(50), timer_callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
