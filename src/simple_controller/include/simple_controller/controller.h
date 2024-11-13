#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <list>
#include <memory>

#include "trajectory_segment.h"

namespace simple_controller {

  using TrajPtr = std::shared_ptr<trajectory::TrajectorySegment>;
/*!
 *\brief 机器人控制器
 * 通过简单的 PID 调节器控制沿着定义的轨迹行驶
 * 角度速度 = pid(error)
 * error是到轨迹的distance
 * 轨迹是angle和linear的列表，保存为指向TrajectorySegement的指针
 * 轨迹是一个圆环
 * 通过ground_truth回调函数接收机器人的反馈数据（机器人的真实位置）
 * 在控制过程中，发布未来轨迹给速度控制器
*/
class Controller : public rclcpp::Node {
protected:
  // 首先声明在构造函数初始化列表中最先初始化的变量
  double p_factor;
  double i_factor;
  double d_factor;
  double max_antiwindup_error;
  double error_integral;
  double last_error;
  double radius;
  double cy;
  double max_curvature;
  double traj_dl;
  double traj_length;

  // 订阅者和发布者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr err_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

  // 其他成员变量
  double robot_x;
  double robot_y;
  double robot_theta = 0.0;
  rclcpp::Time robot_time;
  double current_linear_velocity = 0.0; // 当前线速度
  double current_angular_velocity = 0.0; // 当前角速度
  double lam = 0.1; // 预测系数
  double c = 1; // 预测起点
  std::size_t cal_target_index(); // 获取目标点索引

  // 用列表存储每段轨迹
  using Trajectory  = std::list<TrajPtr>; // ?
  std::list<TrajPtr> trajectory;
  
  nav_msgs::msg::Path::SharedPtr path = std::make_shared<nav_msgs::msg::Path>(); // 初始化 path 变量
  std::size_t nearest_point_index;

  std::list<TrajPtr>::iterator current_segment;
  double current_segment_length = 0.0;
  
  std::string world_frame_id;  // 添加 world_frame_id 成员变量

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(const double yaw) const;
  void on_timer(); //
  // 接受车辆坐标消息
  void on_pose(const nav_msgs::msg::Odometry::SharedPtr odom);
  // 接受路径消息
  // void on_path(const nav_msgs::msg::Path &path);
  void on_path(const nav_msgs::msg::Path::SharedPtr path);
  // 计算轨迹反馈误差
  double cross_track_error();
  void update_robot_pose(double dt);
  void publish_trajectory();
  void on_odo(const nav_msgs::msg::Odometry::SharedPtr odom);
  void publish_error(double error);
  nav_msgs::msg::Path::SharedPtr create_path() const;
  std::size_t get_nearest_path_pose_index(int start_index, std::size_t search_len);

public:
  double get_p_factor() { return p_factor; }
  double get_i_factor() { return i_factor; }
  double get_d_factor() { return d_factor; }
  void reset();
  void reset(double p, double d, double i);
  // 构造函数
  Controller(const std::string &ns = "simple_controller");
  // 虚机构函数,用于在派生类中实现
  virtual ~Controller();

};

}

#endif
