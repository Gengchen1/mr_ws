#include "mpc_controller.h"

#include <angles/angles.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <rclcpp/time.hpp>  // 确保包含时间相关头文件
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/float32.hpp>

#include "rclcpp/rclcpp.hpp"

namespace mpc_controller {

template <class T>
T clip(T val, T max) {
  if (val > max) return max;
  if (val < -max) return -max;
  return val;
}

void MPCController::update_trajectory_segment() {
  // 小车到轨迹段起始点的距离
  current_segment_length =
      (*current_segment)->get_point_length(robot_x, robot_y);
  // <0, 说明小车在轨迹段之前，向前回溯，直到找到小车所在的轨迹上
  while (current_segment_length < 0.0) {
    if (current_segment == trajectory.begin())
      current_segment = trajectory.end();

    --current_segment;
    current_segment_length =
        (*current_segment)->get_point_length(robot_x, robot_y);
  }
  // >0, 说明小车在轨迹段之后，向后回溯，直到找到小车所在的轨迹上
  while (current_segment_length > (*current_segment)->get_length()) {
    ++current_segment;
    if (current_segment == trajectory.end())
      current_segment = trajectory.begin();
    current_segment_length =
        (*current_segment)->get_point_length(robot_x, robot_y);
  }
  //  ROS_DEBUG_STREAM("current segment length "<<current_segment_length);
}

void MPCController::update_control_points() {
  control_points.resize(control_points_num);
  Trajectory::iterator segment = current_segment;
  tf2::Vector3 pose(robot_x, robot_y, 0);
  RCLCPP_DEBUG(this->get_logger(), "control points");
  double control_point_distance =
      (*segment)->get_point_length(pose.x(), pose.y());
  for (std::size_t i = 0; i < control_points_num; ++i) {
    // 获取每个控制点的距离
    control_point_distance += i * control_points_dl;
    // 如果小车当前位置到控制点的距离>当前轨迹段长度,
    // 需要减去当前轨迹长度，移动到下一个轨迹段
    while (control_point_distance > (*segment)->get_length()) {
      control_point_distance -= (*segment)->get_length();
      ++segment;
      if (segment == trajectory.end()) segment = trajectory.begin();
    }
    control_points[i] = (*segment)->get_point(control_point_distance);
    RCLCPP_DEBUG(this->get_logger(), "%d: %f %f", i, control_points[i].x(),
                 control_points[i].y());
  }
}

void MPCController::convert_control_points() {
  tf2::Transform world2robot = robot2world.inverse();
  RCLCPP_DEBUG(this->get_logger(), "control points in robot coordinates ");
  for (auto& point : control_points) {
    // 将每个控制点转换成机器人坐标系
    point = world2robot(point);
    RCLCPP_DEBUG(this->get_logger(), "%f %f", point.x(), point.y());
  }
}

// 计算多项式系数
void MPCController::calculate_control_coefs() {
  const int order = 3;  // 多项式阶数为3
  assert(order <= control_points.size() - 1);
  Eigen::MatrixXd A(control_points.size(), order + 1);

  // 初始化系数矩阵A,用来存储控制点的x坐标的次幂
  for (size_t i = 0; i < control_points.size(); ++i) {
    A(i, 0) = 1.0;
  }
  // 向量yvals用来存储控制点的y坐标
  Eigen::VectorXd yvals(control_points.size());
  for (size_t j = 0; j < control_points.size(); j++) {
    yvals(j) = control_points[j].y();
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * control_points[j].x();
    }
  }
  auto Q = A.householderQr();
  Eigen::VectorXd result = Q.solve(yvals);
  control_coefs.assign(result.data(), result.data() + result.size());
  RCLCPP_DEBUG(this->get_logger(), "coefs: %f %f %f %f", control_coefs[0],
               control_coefs[1], control_coefs[2], control_coefs[3]);
}

// 计算多项式在给定x处的值
double MPCController::polyeval(double x) {
  double result = control_coefs[0];
  double ax = 1.0;
  for (size_t i = 1; i < control_coefs.size(); ++i) {
    ax *= x;
    result += ax * control_coefs[i];
  }
  return result;
}

void MPCController::update_robot_pose(double dt) {
  // 更新机器人当前位置
  robot_x += current_linear_velocity * dt * cos(robot_theta);
  robot_y += current_linear_velocity * dt * sin(robot_theta);

  // 更新机器人当前朝向角
  robot_theta =
      angles::normalize_angle(robot_theta + current_angular_velocity * dt);

  // 更新当前时间，使用节点的时钟
  robot_time = this->now();

  // 将机器人位姿转化成世界坐标系下的向量
  robot2world.setOrigin(tf2::Vector3(robot_x, robot_y, 0));

  // 根据朝向角设置机器人在世界坐标系下的四元数
  robot2world.setRotation(
      tf2::Quaternion(0, 0, sin(robot_theta / 2), cos(robot_theta / 2)));
}

// 发布控制指令
void MPCController::apply_control() {
  // 当前速度
  cmd_vel += cmd_acc * control_dt;
  cmd_steer_angle += cmd_steer_rate * control_dt;
  cmd_steer_angle = clip<double>(cmd_steer_angle, max_steer_angle);
  // 将曲率作为命令发送到驱动器
  auto cmd = std_msgs::msg::Float32();
  cmd.data = cmd_steer_angle;
  steer_pub->publish(cmd);
  // 将速度作为命令发送给驱动器
  cmd.data = cmd_vel;
  vel_pub->publish(cmd);
  RCLCPP_DEBUG(this->get_logger(), "cmd v = %f angle = %f", cmd_vel,
               cmd_steer_angle);
}

void MPCController::on_timer() {
  apply_control();

  // 计算机器人下一个周期的位姿
  double dt = (this->get_clock()->now() - robot_time).seconds();
  update_robot_pose(dt + control_dt);

  // 更新行驶的轨迹段
  update_trajectory_segment();

  // 更新时间控制点
  update_control_points();

  // 转换控制点到世界坐标系
  convert_control_points();

  // 计算轨迹线系数
  calculate_control_coefs();

  // 第一个多项式系数是系统偏移量或初始化误差，常数项
  double error = control_coefs[0];
  RCLCPP_DEBUG(this->get_logger(), "error from coef[0] = %f", error);

  // 开始求解
  const auto start_solve = this->now();

  // 调用 solve 时，确保参数与 MPC::solve 声明一致
  mpc.solve(current_linear_velocity, cmd_steer_angle, control_coefs,
             cmd_steer_rate, cmd_acc, mpc_x, mpc_y);

  // 求解时间
  double solve_time = (this->now() - start_solve).seconds();
  RCLCPP_DEBUG(this->get_logger(), "solve time = %f", solve_time);
  if (solve_time > 0.08) {
    RCLCPP_ERROR(this->get_logger(), "Solve time too big %f", solve_time);
  }
  publish_trajectory();
  publish_poly();
  // 发送误差用于调试
  publish_error(cross_track_error());
  publish_mpc_traj(mpc_x, mpc_y);
}

// 更新机器人位姿，x,y坐标和朝向
void MPCController::on_pose(const nav_msgs::msg::Odometry::SharedPtr odom) {
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_theta = tf2::getYaw(odom->pose.pose.orientation);

  world_frame_id = odom->header.frame_id;
  robot_time = odom->header.stamp;

  //  current_velocity = odom->twist.twist.linear.x;
  //  RCLCPP_DEBUG(this->get_logger(), "x = %f y = %f %f", robot_x, robot_y,
  //  robot_theta);

  //  RCLCPP_DEBUG(this->get_logger(), "truth vel = %f",
  //  odom->twist.twist.linear.x);
}

// 根据里程计信息获取当前速度，和当前转向角
void MPCController::on_odo(const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_linear_velocity = odom->twist.twist.linear.x;
  current_angular_velocity = odom->twist.twist.angular.z;
  if (std::abs(current_linear_velocity) > 0.01) {
    current_curvature = current_angular_velocity / current_linear_velocity;
    current_angle = atan(current_curvature * wheel_base);
  } else {
    current_curvature = 0.0;
    current_angle = 0.0;
  }
  RCLCPP_DEBUG(this->get_logger(), "odom vel = %f w = %f angle = %f",
               current_linear_velocity, current_angular_velocity,
               current_angle);
}

void MPCController::publish_error(double error) {
  auto err_msg = std_msgs::msg::Float32();
  err_msg.data = error;
  err_pub->publish(err_msg);
}

double MPCController::cross_track_error() {
  double error = 0.0;
  if (robot_y < radius) {
    double rx = robot_x;
    double ry = robot_y - radius;
    error = sqrt(rx * rx + ry * ry) - radius;
  } else if (robot_y > cy) {
    double rx = robot_x;
    double ry = robot_y - cy;
    error = sqrt(rx * rx + ry * ry) - radius;
  } else if (robot_x > 0) {
    error = robot_x - radius;
  } else if (robot_x < 0) {
    error = -radius - robot_x;
  }
  return error;
}

void MPCController::get_segment(
    std::list<TrajPtr>::iterator& traj_it,
    [[maybe_unused]] double& len) {  // 标记未使用的参数
  // 初始化为轨迹最后段
  traj_it = trajectory.end();
  // 机器人所在位置符合条件时，赋值为初始轨迹段
  if (robot_y < radius) {
    if (robot_x >= 0) {
      traj_it = trajectory.begin();
    }
  }
}

void add_point(sensor_msgs::msg::PointCloud::SharedPtr msg,
               const tf2::Vector3& point) {
  geometry_msgs::msg::Point32 p;
  // 将三维向量转化成位置信息的点
  p.x = point.x();
  p.y = point.y();
  p.z = point.z();
  msg->points.push_back(p);
}

// 发布轨迹点云信息
void MPCController::publish_trajectory() {
  // prepare pointcloud message
  sensor_msgs::msg::PointCloud::SharedPtr msg =
      std::make_shared<sensor_msgs::msg::PointCloud>();
  // 帧ID
  msg->header.frame_id = world_frame_id;
  // 时间戳
  msg->header.stamp = robot_time;
  // 点的数量
  int trajectory_points_quantity = traj_length / traj_dl + 1;
  // 剩余存储消息的空间
  int points_left = trajectory_points_quantity;
  msg->points.reserve(trajectory_points_quantity);
  // 移除未使用的变量
  // double publish_len = 0;
  Trajectory::iterator it = current_segment;
  double start_segment_length = current_segment_length;

  while (points_left) {
    double segment_length = (*it)->get_length();
    // 计算当前可以添加的点数
    int segment_points_quantity = std::min<int>(
        points_left, floor((segment_length - start_segment_length) / traj_dl));
    // 将当前所有的点添加到消息中
    for (int i = 0; i <= segment_points_quantity; ++i) {
      add_point(msg, (*it)->get_point(start_segment_length + i * traj_dl));
    }
    points_left -= segment_points_quantity;
    // 如果还有剩余点，就切换到下一段
    if (points_left) {
      // 下一段的起点
      start_segment_length +=
          (segment_points_quantity + 1) * traj_dl - segment_length;
      // 移动到下一个迭代器，如果到达轨迹末尾，则重置为轨迹起点
      ++it;
      if (it == trajectory.end()) it = trajectory.begin();
    }
  }
  // 发布消息
  traj_pub->publish(*msg);
}

// 预测控制点云间距
void MPCController::publish_poly() {
  // prepare pointcloud message
  sensor_msgs::msg::PointCloud::SharedPtr msg =
      std::make_shared<sensor_msgs::msg::PointCloud>();  // 初始化 msg
  msg->header.frame_id = world_frame_id;
  msg->header.stamp = robot_time;
  // x轴的范围
  double xrange = control_points_dl * control_points_num * 1.5;
  // 轨迹点数量
  int trajectory_points_quantity = static_cast<int>(xrange / traj_dl);
  msg->points.reserve(trajectory_points_quantity);

  for (int i = 0; i < trajectory_points_quantity; ++i) {
    // 当前要计算的点坐标x
    double x = i * traj_dl;
    // 根据轨迹多项式计算当前点的坐标信息
    tf2::Vector3 point = robot2world(tf2::Vector3(x, polyeval(x), 0));
    add_point(msg, point);
  }
  poly_pub->publish(*msg);
}

void MPCController::publish_mpc_traj(std::vector<double>& x,
                                     std::vector<double>& y) {
  // 根据计算的一系列参考坐标来发布轨迹
  if (x.empty()) return;
  sensor_msgs::msg::PointCloud::SharedPtr msg =
      std::make_shared<sensor_msgs::msg::PointCloud>();  // 初始化 msg
  msg->header.frame_id = world_frame_id;
  msg->header.stamp = robot_time;
  msg->points.reserve(x.size());
  for (size_t i = 0; i < x.size(); ++i) {
    add_point(msg, robot2world(tf2::Vector3(x[i], y[i], 0)));
  }
  mpc_traj_pub->publish(*msg);
}

/*!
 *\简短的构造函数
 *从ns加载参数
 *比例、微分、积分 -pid 因子
 *max_antiwindup_error -使用积分组件的最大误差
 *轨迹由两条线连接的两个圆段组成
 *第一个圆心为 (0, radius)，第二个圆心为 (0, cy)
 *radius -圆形零件的半径
 *cy -第二个圆的中心
 *traj_dl -已发布轨迹的离散
 *traj_length -已发布轨迹的长度
 *timer_period -离散定时器
 */
MPCController::MPCController(const std::string& ns)
    : Node(ns),
      radius(this->declare_parameter("radius", 10.0)),
      cy(this->declare_parameter("cy", 20.0)),  // 2 * radius
      wheel_base(this->declare_parameter("wheel_base", 1.88)),
      max_steer_angle(this->declare_parameter("max_steer_angle", 0.3)),
      max_steer_rate(this->declare_parameter("max_steer_rate", 0.3)),
      max_velocity(this->declare_parameter("max_velocity", 5.0)),
      max_acc(this->declare_parameter("max_acc", 0.8)),
      control_dt(this->declare_parameter("timer_period", 0.1)),
      traj_dl(this->declare_parameter("traj_dl", 0.2)),
      traj_length(this->declare_parameter("traj_length", 5.0)),
      mpc_steps(this->declare_parameter("mpc_steps", 4)),
      mpc_dt(this->declare_parameter("mpc_dt", 0.5)),
      mpc(mpc_steps, mpc_dt, max_velocity, max_acc,
          this->declare_parameter("max_delta", 0.3), max_steer_rate, wheel_base,
          this->declare_parameter("kcte", 0.0),
          this->declare_parameter("kepsi", 0.0),
          this->declare_parameter("kev", 0.0),
          this->declare_parameter("ksteer_cost", 0.0)),
      robot_time(this->now()),
      timer(this->create_wall_timer(std::chrono::duration<double>(control_dt),
                                    std::bind(&MPCController::on_timer, this))),
      pose_sub(this->create_subscription<nav_msgs::msg::Odometry>(
          "~/ground_truth", 1,
          std::bind(&MPCController::on_pose, this, std::placeholders::_1))),
      odo_sub(this->create_subscription<nav_msgs::msg::Odometry>(
          "~/odom", 1,
          std::bind(&MPCController::on_odo, this, std::placeholders::_1))),
      err_pub(this->create_publisher<std_msgs::msg::Float32>("error", 10)),
      steer_pub(this->create_publisher<std_msgs::msg::Float32>("/steering", 1)),
      vel_pub(this->create_publisher<std_msgs::msg::Float32>("/velocity", 1)),
      traj_pub(this->create_publisher<sensor_msgs::msg::PointCloud>(
          "trajectory", 1)),
      poly_pub(this->create_publisher<sensor_msgs::msg::PointCloud>("poly", 1)),
      mpc_traj_pub(this->create_publisher<sensor_msgs::msg::PointCloud>("mpc_traj", 1))
{
  // counter clock
  trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(
      1.0 / radius, 0, 0, 1.0, 0, M_PI / 2 * radius));
  trajectory.emplace_back(std::make_shared<trajectory::LinearSegment>(
      radius, radius, 0.0, 1.0, cy - radius));
  trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(
      1.0 / radius, radius, cy, 0.0, 1.0, M_PI / 2 * radius));
  trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(
      1.0 / radius, 0, radius + cy, -1.0, 0.0, M_PI / 2 * radius));
  trajectory.emplace_back(std::make_shared<trajectory::LinearSegment>(

      -radius, cy, 0.0, -1.0, cy - radius));
  trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(
      1.0 / radius, -radius, radius, 0.0, -1.0, M_PI / 2 * radius));

  current_segment = trajectory.begin();
}

MPCController::~MPCController() {
  // TODO Auto-generated destructor stub
}

}  // namespace mpc_controller
