#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>

// 全局变量
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
std::string map_frame;
double map_resolution = 0.1;
int map_width = 2000;
int map_height = 2000;
nav_msgs::msg::OccupancyGrid map_msg;

// 准备地图消息
void prepareMapMessage(nav_msgs::msg::OccupancyGrid &map_msg) {
    map_msg.header.frame_id = map_frame;
    map_msg.info.height = map_height;
    map_msg.info.width = map_width;
    map_msg.info.resolution = map_resolution;
    map_msg.info.origin.position.x = -map_width * map_resolution / 2.0;
    map_msg.info.origin.position.y = -10.0;
    map_msg.info.origin.orientation.w = 1.0;
    map_msg.data.resize(map_height * map_width, -1);
}

// 计算对数概率和概率值
float log2p(float p) {
    return log(p / (1 - p));
}

float calc_p(float l) {
    return 1 - (1 / (1 + exp(l)));
}

int get_map(float l) {
    float p = calc_p(l);
    if (p < 0.4) {
        return 0;
    } else if (p > 0.6) {
        return 100;
    } else {
        return 50;
    }
}

bool determineScanTransform(geometry_msgs::msg::TransformStamped &scan_transform, const std::string &laser_frame, rclcpp::Time stamp) {
    try {
        // 设置容忍时间为 0.2 秒
        scan_transform = tf_buffer->lookupTransform(map_frame, laser_frame, stamp, tf2::durationFromSec(0.2));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("simple_map"), "Could not transform %s to %s: %s", laser_frame.c_str(), map_frame.c_str(), ex.what());
        return false;
    }
    return true;
}

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub) {
    geometry_msgs::msg::TransformStamped scan_transform;
    const std::string &laser_frame = scan->header.frame_id;
    rclcpp::Time laser_stamp = scan->header.stamp;

    // 打印时间戳以调试时间同步问题
    RCLCPP_INFO(rclcpp::get_logger("simple_map"), "Laser timestamp: %.3f", laser_stamp.seconds());

    if (!determineScanTransform(scan_transform, laser_frame, laser_stamp)) {
        return;
    }

    map_msg.header.stamp = laser_stamp;

    tf2::Vector3 zero_pose(0, 0, 0);
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(scan_transform.transform.translation.x,
                                     scan_transform.transform.translation.y,
                                     scan_transform.transform.translation.z));
    transform.setRotation(tf2::Quaternion(scan_transform.transform.rotation.x,
                                          scan_transform.transform.rotation.y,
                                          scan_transform.transform.rotation.z,
                                          scan_transform.transform.rotation.w));
    tf2::Vector3 scan_pose = transform * zero_pose;

    int y = (scan_pose.y() - map_msg.info.origin.position.y) / map_resolution;
    int x = (scan_pose.x() - map_msg.info.origin.position.x) / map_resolution;

    if (y >= 0 && y < map_height && x >= 0 && x < map_width) {
        map_msg.data[y * map_width + x] = 0;
    }

    int size_ranges = scan->ranges.size();
    float curr_angle = scan->angle_min;
    float delta_angle = scan->angle_increment;

    for (int i = 0; i < size_ranges; ++i) {
        float range = scan->ranges[i];
        if (range > scan->range_min && range < scan->range_max) {
            float h = scan->range_min;
            while (h <= range) {
                tf2::Vector3 h_pose(h * cos(curr_angle), h * sin(curr_angle), 0);
                tf2::Vector3 h_map = transform * h_pose;

                int h_y = (h_map.y() - map_msg.info.origin.position.y) / map_resolution;
                int h_x = (h_map.x() - map_msg.info.origin.position.x) / map_resolution;

                if (h_y >= 0 && h_y < map_height && h_x >= 0 && h_x < map_width) {
                    float p = 0.5;
                    if (fabs(range - h) < 0.1) {
                        p = 1.0;
                    } else if (range - h > 0.1) {
                        p = 0.0;
                    }

                    float log_prev = log2p(float(map_msg.data[h_y * map_width + h_x]) / 100);
                    float log_free = log2p(0.5);
                    float log_inv = log2p(p);
                    float log_ti = log_inv + log_prev - log_free;
                    map_msg.data[h_y * map_width + h_x] = get_map(log_ti);
                }

                h += 0.01;
            }
        }
        curr_angle += delta_angle;
    }
    map_pub->publish(map_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("simple_map");

    map_frame = node->declare_parameter("map_frame", "odom");
    map_resolution = node->declare_parameter("map_resolution", map_resolution);
    map_width = node->declare_parameter("map_width", map_width);
    map_height = node->declare_parameter("map_height", map_height);

    prepareMapMessage(map_msg);

    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/simple_map", 10);
    auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [map_pub](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            laserCallback(msg, map_pub);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


