/*
 * Controller.cpp
 *
 *  Created on: 30 апр. 2017 г.
 *      Author: aminin
 */

#include "controller.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>  // 修改为 Float32
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>  // 添加缺失的头文件

namespace simple_controller {

template <class T>
T clip(T val, T max)
{
  if ( val > max )
    return max;
  if ( val < -max)
    return -max;
  return val;
}

// 根据当前速度和上次小车的位置还有时间间隔更新小车位置
void Controller::update_robot_pose(double dt)
{
  RCLCPP_INFO(this->get_logger(), "update_robot_pose %f v = %f", dt, current_linear_velocity);
  // 更新 x, y 坐标
  robot_x += current_linear_velocity * dt * sin(robot_theta);
  robot_y += current_linear_velocity * dt * cos(robot_theta);
  // 计算当前角度
  robot_theta = angles::normalize_angle(robot_theta + current_angular_velocity * dt);
  // 直接使用节点的时钟更新时间
  // robot_time = this->now();
  robot_time += rclcpp::Duration::from_seconds(dt);
}

// 打印目前走到了哪里，并将当前路径赋给对象成员path
void Controller::on_path(const nav_msgs::msg::Path::SharedPtr path) {
  // %zu代表 size_t类型
  RCLCPP_INFO(this->get_logger(), "Got path %zu", path->poses.size());
  this->path = path;
  nearest_point_index = 0;
  // target_point_index = 0;
}

// 获取相对于当前点最近位置的点索引
std::size_t Controller::get_nearest_path_pose_index(int start_index, std::size_t search_len) {
  double nearest_distance = 1e10; // 1x10的10次方
  std::size_t nearest_index = 0;
  geometry_msgs::msg::Pose nearest_pose;
  // 遍历从起始点到指定长度内的所有点
  for (int index = start_index; index < start_index + static_cast<int>(search_len); ++index) {
    std::size_t real_index;
    // 一个位姿可能包含位置(x, y, z), 和姿态(四元数：x, y, z, w)
    // poses.size()表示包含几个位姿
    // 限定 index在位姿数范围内
    if (index >= 0 && index < static_cast<int>(path->poses.size())) {
      real_index = static_cast<std::size_t>(index);
    }
    if (index < 0) {
      // index < 0, 更新为位姿数减去index数
      real_index = (static_cast<int>(path->poses.size()) + index);
    }
    if (index >= static_cast<int>(path->poses.size())) {
      real_index = static_cast<std::size_t>(index) - path->poses.size();
    }
    // 当前点的位置
    const auto &path_point = path->poses[real_index].pose.position;
    double dx = robot_x - path_point.x;
    double dy = robot_y - path_point.y;
    double distance_sqr = dx * dx + dy * dy;
    // 找到给定长度内最近的距离和最近距离对应的点
    if (distance_sqr < nearest_distance) {
      nearest_distance = distance_sqr;
      nearest_index = real_index;
    }
  }
  return nearest_index;
}

std::size_t Controller::cal_target_index() {
  // 使用固定的小时间间隔而不是计算时间差
  update_robot_pose(0.01);  // 使用10ms作为固定更新间隔
  // update_robot_pose((this->now() - robot_time).seconds());
  nearest_point_index = get_nearest_path_pose_index(nearest_point_index - 10, 20);
  // 获取当前最近点的位姿和角度
  const auto& nearest_pose = path->poses[nearest_point_index].pose;
  const auto& nearest_pose_angle = tf2::getYaw(nearest_pose.orientation);

  double dx = robot_x - nearest_pose.position.x;
  double dy = robot_y - nearest_pose.position.y;
  // 计算误差，误差是机器人当前位置在最近点坐标系下的y轴负方向上的投影
  double error = -(-dx * sin(nearest_pose_angle) + dy * cos(nearest_pose_angle));
  RCLCPP_INFO(this->get_logger(), "error %lf", error);

  double m_error = abs(error); // 取误差绝对�����
  // 打印当前速度
  RCLCPP_DEBUG(this->get_logger(), "current_linear_velovity %lf", current_linear_velocity);
  // 预测前方一小段距离
  double l_d = lam * current_linear_velocity + c;
  // 
  while (l_d > m_error && (nearest_point_index + 1) < path->poses.size()) {
    // 获取下一个的位姿和朝向角度
    const auto &nearest_pose_next = path->poses[nearest_point_index + 1].pose;
    const auto &nearest_pose_angle_next = tf2::getYaw(nearest_pose_next.orientation);

    // 计算机器人当前位置与下一个点位置的差值
    double dx_next = robot_x - nearest_pose_next.position.x;
    double dy_next = robot_y - nearest_pose_next.position.y;

    // 计算下一个点的误差
    double m_error_next = abs(-dx_next * sin(nearest_pose_angle_next) + dy_next * cos(nearest_pose_angle_next));
    // 更新误差和最近索引
    nearest_point_index += 1;
    m_error = m_error_next;
  }
  return nearest_point_index;
}

// 定时更新机器人状态，并根据当前路径和机器人位置计算控制命令，以调整机器人的运动轨迹
void Controller::on_timer() {
  if (std::abs(current_linear_velocity) < 0.01) {
    return;  // 如果速度小于0.01,接近停止，就不再更新状态
  }
  // 更新当前机器��位��
  update_robot_pose((this->now() - robot_time).seconds());
  // 获取最近目标点
  nearest_point_index = cal_target_index();
  // 获取最近目标点位置和朝向角
  const auto& nearest_pose = path->poses[nearest_point_index].pose;
  const auto& nearest_pose_angle = tf2::getYaw(nearest_pose.orientation);
  double dx = robot_x - nearest_pose.position.x;
  double dy = robot_y - nearest_pose.position.y;
  
  // 当前点在以最近点为坐标系上的Y轴上的误差
  double error = -(-dx * sin(nearest_pose_angle) + dy * cos(nearest_pose_angle));
  RCLCPP_INFO(this->get_logger(), "r_error_next %lf", error);

  double l_d = lam * current_linear_velocity + c;
  // 当前角度和目标角度的差值
  double base2Nearest_pose_angle = (nearest_pose_angle - robot_theta);
  // ??? 
  double angular_m_cmd = atan2(2 * 1.88 * sin(base2Nearest_pose_angle), l_d);
  RCLCPP_INFO(this->get_logger(), "angular %lf", angular_m_cmd);
  // 曲率
  double curvature = angular_m_cmd / current_linear_velocity;
  std_msgs::msg::Float32 cmd;  // 修改为 Float32
  // 确保不超过最大曲率
  cmd.data = clip<double>(curvature, max_curvature);
  // 发布移动控制命令
  steer_pub->publish(cmd);
  // 发布轨迹
  publish_trajectory(); 
  // 发布误差信息
  publish_error(error);
  RCLCPP_INFO(this->get_logger(), "steering cmd = %lf", curvature);
}

void Controller::on_pose(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  // 使用 tf2::getYaw() 获取机器人的朝向角
  // robot_theta = 2*atan2(odom->pose.pose.orientation.z,
  //                     odom->pose.pose.orientation.w);  
  robot_theta = tf2::getYaw(odom->pose.pose.orientation);

  // 物理环境ID
  world_frame_id = odom->header.frame_id;  // 确保 world_frame_id 已声明
  // 使用节点的时钟
  robot_time = this->now();
  // robot_time = odom->header.stamp;
}

// 根据里程测量数据更新线速度和角速度
void Controller::on_odo(nav_msgs::msg::Odometry::SharedPtr odom) {
  current_linear_velocity = odom->twist.twist.linear.x;
  current_angular_velocity = odom->twist.twist.angular.z;
  // RCLCPP_DEBUG(this->get_logger(), "odom vel = ", current_velocity);
}

// 发布错误信息
void Controller::publish_error(double error) {
  std_msgs::msg::Float32 err_msg;  // 修改为 Float32
  err_msg.data = error;
  err_pub->publish(err_msg);
}

// 横向跟踪误差，小车中心点到最近路径点的距离
double Controller::cross_track_error() {
  double error = 0.0;
  // 计算车辆到轨迹圆��缘的横向偏差，横向偏差为0，代表恰好在轨迹上
  // 横向偏差为负，代表在轨迹内，横向偏差为正，代表在轨迹外
  if (robot_y < radius) {
    double rx = robot_x;
    double ry = robot_y - radius;
    error = sqrt(rx * rx + ry * ry) - radius;
    // 计算车辆到轨迹圆心的横向偏差，横向偏差为0，代表恰好在轨迹上
    // 横向偏差为负，代表在轨迹外，横向偏差为正，代表在轨迹内
  } else if ( robot_y > cy ){
    double rx = robot_x;
    double ry = robot_y - cy;
    error = sqrt(rx * rx + ry * ry) - radius;
  } else if ( robot_x > 0 ) {
    error = robot_x - radius;
  } else if ( robot_x < 0 ) {
    error = -radius - robot_x;
  }
  return error;
}

void Controller::publish_trajectory()
{
  RCLCPP_DEBUG(this->get_logger(), "Publish trajectory!");
 //  ROS_DEBUG_STREAM("publish trajectory");
  path_pub->publish(*path);
}

void Controller::reset()
{
  error_integral = 0.0;
  last_error = cross_track_error();
}

void Controller::reset(double p, double d, double i)
{
  reset();
  p_factor = p;
  d_factor = d;
  i_factor = i;
}

// 根据机器人当前轨迹创建并发布一条路径
nav_msgs::msg::Path::SharedPtr Controller::create_path() const {
  // 初始化 path
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.frame_id = "odom"; // 设置路径的坐标系
  // 使用节点的当前时间
  path->header.stamp = this->now();
  // path->header.stamp = robot_time;
  auto segment_it = trajectory.begin(); // 获取轨迹段的迭代器
  std::size_t points_added = 0; // 初始化已添加的点数
  double point_length = 0.0; // 初始化点的长度

  while (segment_it != trajectory.end()) { // 遍历所有轨迹段
    const auto segment = *segment_it; // 获取当前轨迹段
    double segment_length = segment->get_length(); // 获取当前轨迹段的长度
    while (point_length <= segment_length) { // 遍历当前轨迹段内的所有点
      const auto point = segment->get_point(point_length); // 获取当前长度的点
      const auto angle = segment->get_orientation(point_length); // 获取当前长度的点的朝向角
      geometry_msgs::msg::PoseStamped pose; // 创建位姿消息对象
      pose.header.frame_id = "odom"; // 设位姿的坐标系
      pose.pose.position.x = point.x(); // 设置位姿的x坐标
      pose.pose.position.y = point.y(); // 设置位姿的y坐标
      pose.pose.orientation = createQuaternionMsgFromYaw(angle); // 设置位姿的朝向
      path->poses.push_back(pose); // 将位姿添加到路径中
      point_length += traj_dl; // 增加点的长度,获取下一个长度出的点
      points_added++; // 增加已添加的点数
    }
    point_length -= segment_length; // 减去当前轨迹段的长度? 相当于至0吧，因为退出while时point_length == segment_length
    ++segment_it; // 移动到下一个轨迹段
  }
  return path; // 返回生成的路径
}
// ros2中没有yaw角和四元数的转换函数，自己写了一个。
geometry_msgs::msg::Quaternion Controller::createQuaternionMsgFromYaw(const double yaw) const {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

/*!
 *\ 简介 构造函数
 * 从ns加载参数
 * 比例、微分、积分 -pid 因子
 * max_antiwindup_error - 最大抗积分饱和，pid中的i
 * 轨迹由两条线连接的两个圆段组成
 * 第一个圆心(0, radius)，第二个圆心为 (0, cy)
 * radius -圆形零件的半径
 * cy -第二个圆中心
 * traj_dl -已发布轨迹的离散长度
 * traj_length -已发布轨迹的长度
 * timer_period -离散定时器
 */
Controller::Controller(const std::string& ns, const std::string &namespace_)
   : Node(ns,namespace_),
  // 声明PID三个参数的名字和默认值
  p_factor(this->declare_parameter("proportional", 1.0)),
  i_factor(this->declare_parameter("integral", 0.0)),
  d_factor(this->declare_parameter("differential", 0.0)),
  max_antiwindup_error(this->declare_parameter("max_antiwindup_error", 0.5)),
  error_integral(0.0),
  last_error(0.0),
  radius(this->declare_parameter("radius", 10.0)),
  cy(this->declare_parameter("cy", 2 * radius)),
  max_curvature(this->declare_parameter("max_curvature", 0.2)),
  traj_dl(this->declare_parameter("traj_dl", 0.2)),
  traj_length(this->declare_parameter("traj_length", 5.0)),
  pose_sub(this->create_subscription<nav_msgs::msg::Odometry>(
    "~/ground_truth", 1, std::bind(&Controller::on_pose, this, std::placeholders::_1))),
  odo_sub(this->create_subscription<nav_msgs::msg::Odometry>(
    "~/odom", 1, std::bind(&Controller::on_odo, this, std::placeholders::_1))),
  path_sub(this->create_subscription<nav_msgs::msg::Path>(
    "~/path", 1, std::bind(&Controller::on_path, this, std::placeholders::_1))),
  // 创建一个墙上定时器(以实际时间为准),时间间隔设置为0.1秒，每0.1秒调用一次回调函数
  timer(this->create_wall_timer(
    std::chrono::duration<double>(this->declare_parameter("timer_period", 0.1)),
    std::bind(&Controller::on_timer, this))),
  err_pub(this->create_publisher<std_msgs::msg::Float32>("~/error", 10)),  // 修改为 Float32
  steer_pub(this->create_publisher<std_msgs::msg::Float32>("/steering", 10)),  // 修改为 Float32
  path_pub(this->create_publisher<nav_msgs::msg::Path>("~/controller_path", 1)),
  robot_time(this->now()) // 使用节点的当前时间初始化
{
  // 顺时针轨迹,  cy 是第二个圆心的y轴坐标
  // 曲率，起始点坐标 x, y,  起始点切向量坐标 vx, vy, 轨迹段长度
  // 前1/4个圆
  trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(
    1.0 / radius, 0, 0, 1.0, 0, M_PI / 2 * radius));
  // 起始点 x, y 切向量 vx, vy, 轨迹段长度
  // 第二条直线
  trajectory.emplace_back(std::make_shared<trajectory::LinearSegment>(
    radius, radius, 0.0, 1.0, cy - radius));
  // 第二个1/4圆
  trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(
    1.0 / radius, radius, cy, 0.0, 1.0, M_PI / 2 * radius));
  // 第三个1/4圆
  trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(
    1.0 / radius, 0, radius + cy, -1.0, 0.0, M_PI / 2 * radius));
  // 第二条直线
  trajectory.emplace_back(std::make_shared<trajectory::LinearSegment>(
    -radius, cy, 0.0, -1.0, cy - radius));
  // 第四个1/4圆
  trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(
    1.0 / radius, -radius, radius, 0.0, -1.0, M_PI / 2 * radius));

  // 一共六个segment
  current_segment = trajectory.begin();
  const auto trajectory_path = create_path();
  on_path(trajectory_path);

} // 补上缺失的构造函数结束大括号

// 添加析构函数的定义
Controller::~Controller()
{
  // 析构函数内容，如果无需特殊处理，可以留空
}

} // 补上缺失的命名空间结束大括号