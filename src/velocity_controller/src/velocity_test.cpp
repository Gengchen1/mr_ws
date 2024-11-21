#include <algorithm>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class Test : public rclcpp::Node {
 private:
  double desired_velocity = 0;
  double max_velocity = 10;
  double acc = 1.0;
  double max_test_time = 5.0;
  double acc_time = max_test_time / 2;  // 加速时间
  double dcc_time = acc_time;           // 减速时间
  bool started = false;
  // double current_velocity;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_pub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;

 public:
  rclcpp::Time start_test_time;

  void on_odo(const nav_msgs::msg::Odometry::SharedPtr odom) {
    double current_velocity = odom->twist.twist.linear.x;
  }

  void on_timer() {
    const auto time = this->now();
    if (!started) {
      start_test_time = time;
      started = true;
    }
    auto test_time = (time - start_test_time).seconds();

    std_msgs::msg::Float32 vcmd;
    if (test_time >= max_test_time) {
      desired_velocity = 0;
    } else {
      if (test_time <= acc_time) {
        desired_velocity = std::min(max_velocity, test_time * acc);
      } else {
        if (test_time >= dcc_time) {
            desired_velocity = std::max(0.0, max_velocity - acc * (test_time - dcc_time));
        } else {
          desired_velocity = max_velocity;
        }
      }
    }

    if (desired_velocity > 0.01) {
      RCLCPP_INFO(this->get_logger(), "test_time = %F, test_vel = %F",
                  test_time, desired_velocity);
    }
    vcmd.data = desired_velocity;
    test_pub->publish(vcmd);
  }

  Test(const std::string &name)
      : Node(name),
        desired_velocity(0),
        max_velocity(this->declare_parameter<double>("max_velocity", 7.0)),
        acc(this->declare_parameter<double>("acc", 1.0)),
        max_test_time(this->declare_parameter<double>("test_time", 10.0)),
        acc_time(max_test_time / 2),
        dcc_time(acc_time),
        started(false),
        test_pub(this->create_publisher<std_msgs::msg::Float32>("velocity", 1)),
        timer(this->create_wall_timer(std::chrono::duration<double>(0.1),
                                      std::bind(&Test::on_timer, this))),
        odo_sub(this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 1, std::bind(&Test::on_odo, this, std::placeholders::_1))) {
    if (max_velocity / acc > max_test_time / 2.0) {
      acc_time = max_test_time / 2.0;
      dcc_time = max_test_time / 2.0;
      max_velocity = acc * max_test_time / 2.0;
      RCLCPP_WARN(this->get_logger(), "Not enough time to reach max velocity %F", max_velocity);
    } else {
      acc_time = max_velocity / acc;
      dcc_time = max_test_time - acc_time;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Test>("velocity_test");
  if (node->declare_parameter("/use_sim_time", false)) {
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      node->start_test_time = node->now();
      if (node->start_test_time.seconds() != 0.0) {
        break;
      }
    }
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
