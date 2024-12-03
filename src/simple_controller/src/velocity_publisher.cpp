#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class VelocityPublisher : public rclcpp::Node {
public:
    VelocityPublisher() : Node("velocity_publisher") {
        this->declare_parameter<double>("velocity_value", 2.0);
        this->declare_parameter<double>("publish_rate", 1.0);

        auto velocity = this->get_parameter("velocity_value").as_double();
        auto rate = this->get_parameter("publish_rate").as_double();

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/robot/velocity", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / rate)),
            [this, velocity]() {
                auto message = std_msgs::msg::Float32();
                message.data = static_cast<float>(velocity);
                publisher_->publish(message);
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}
