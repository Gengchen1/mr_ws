#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <random>

double car_length = 1.5;
double steering = 0;
double cmd_steering = 0.0;
double max_steering_rate = 1.0;
double max_steering = 0.5;

double throttle = 0;
double velocity = 0;
double cmd_throttle = 0;
double max_throttle_rate = 200.0;
double max_throttle = 100;
double max_velocity = 20;

double kMass = 500;
double kFriction = 1.0;
double kWindFriction = 0.01;
double kBrake = 3.0;
double kThrottle = 2.5;
double kVelExp = 0.8;
double velocity_noise = 0.0;

std::default_random_engine noise_generator;
std::normal_distribution<double> noise_distr;

class StageThrottleController : public rclcpp::Node {
public:
    StageThrottleController() : Node("stage_throttle_controller") {
        // Загрузка параметров
        car_length = this->declare_parameter("length", 1.5);
        max_steering = this->declare_parameter("max_steering", 0.5);
        max_steering_rate = this->declare_parameter("max_steering_rate", 1.0);
        max_velocity = this->declare_parameter("max_velocity", 15.0);
        max_throttle_rate = this->declare_parameter("max_throttle_rate", 200.0);
        max_throttle = this->declare_parameter("max_throttle", 100.0);
        velocity_noise = this->declare_parameter("velocity_noise", 0.0);
        if (velocity_noise != 0.0) {
            noise_distr = std::normal_distribution<double>(0.0, velocity_noise);
        }

        kMass = this->declare_parameter("mass", 500.0);
        kFriction = this->declare_parameter("friction", 1.0);
        kWindFriction = this->declare_parameter("wind_friction", 0.01);
        kBrake = this->declare_parameter("brake", 3.0);
        kThrottle = this->declare_parameter("throttle", 2.5);
        kVelExp = this->declare_parameter("exp", 0.8);

        // Инициализация издателей и подписчиков
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        real_throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("realized_throttle", 1);
        real_steer_pub_ = this->create_publisher<std_msgs::msg::Float32>("realized_steering", 1);

        steer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "steering", 1, std::bind(&StageThrottleController::on_steering, this, std::placeholders::_1));
        throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "throttle", 1, std::bind(&StageThrottleController::on_throttle, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&StageThrottleController::on_timer, this));
        
        last_timer_time_ = this->now();
    }

private:
    void on_steering(const std_msgs::msg::Float32::SharedPtr msg) {
        cmd_steering = msg->data;
    }

    void on_throttle(const std_msgs::msg::Float32::SharedPtr msg) {
        cmd_throttle = msg->data;
    }

    double clamp(double cmd, double max_value) {
        return copysign(std::min(std::abs(cmd), max_value), cmd);
    }

    double clamp(double value, double cmd, double max_value, double max_rate, double dt) {
        cmd = clamp(cmd, max_value);
        auto diff = (cmd - value);
        diff = clamp(diff, max_rate * dt);
        return value + diff;
    }

    void publish(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub, double data) {
        auto msg = std_msgs::msg::Float32();
        msg.data = data;
        pub->publish(msg);
    }

    double acc_from_throttle(double dt) {
        throttle = clamp(throttle, cmd_throttle, max_throttle, max_throttle_rate, dt);
        publish(real_throttle_pub_, throttle);
        double throttle_force = throttle > 0 ? kThrottle * throttle * exp(-velocity * kVelExp) : kBrake * throttle;
        double acc = 1.0 / kMass * (throttle_force - velocity * velocity * kWindFriction - kFriction * velocity);
        return acc;
    }

    void on_timer() {
        auto t = this->now();
        auto dt = (t - last_timer_time_).seconds();
        last_timer_time_ = t;

        velocity = std::max(0.0, clamp(velocity + acc_from_throttle(dt) * dt, max_velocity));
        double send_velocity = velocity;
        
        if (velocity_noise != 0.0 && std::abs(velocity) > 0.01) {
            send_velocity = std::max<double>(0.0, send_velocity + noise_distr(noise_generator));
        }

        steering = clamp(steering, cmd_steering, max_steering, max_steering_rate, dt);
        publish(real_steer_pub_, steering);

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = send_velocity;
        cmd.angular.z = send_velocity * tan(steering) / car_length;
        twist_pub_->publish(cmd);
    }

    // ROS 2 узлы и таймеры
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr real_throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr real_steer_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_timer_time_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StageThrottleController>());
    rclcpp::shutdown();
    return 0;
}
