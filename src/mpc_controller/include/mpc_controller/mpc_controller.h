#ifndef MPC_CONTROLLER_H_
#define MPC_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <list>
#include <memory>
#include <acado/acado_toolkit.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include "trajectory_segment.h"
#include "mpc.h"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include <rclcpp/time.hpp>  // 确保包含时间相关头文件
#include <memory>  // 添加智能指针支持

// ...existing code...
namespace mpc_controller
{
// ...existing code...
using TrajPtr = std::shared_ptr<trajectory::TrajectorySegment>;

/*!
 *\���要机器人控制器
 *通过简单的 PID 调节器控制沿着��义的轨迹行驶
 *角度速度 = pid(错误)
 *误��是到轨迹的���离
 *轨迹是角度和线段的列表，保存为指向基���轨迹的指针
 *轨迹循环
 *通过ground_truth回调接收机器人的反馈（机器���的真实位置）
 *在控制过程中，为速度����制器发布未来轨迹
 */
class MPCController : public rclcpp::Node
{
protected:
  // 先声明基本类型成员变量，这些变量在初始化其他成员变量时会被使用
  double radius;
  double cy;
  double wheel_base;
  double max_steer_angle;
  double max_steer_rate;
  double max_velocity;
  double max_acc;
  double control_dt;
  double traj_dl;
  double traj_length;
  double mpc_steps;
  double mpc_dt;

  // 接着声明复杂类型成员变量，这些成员变量的初始化可能依赖于上述基本类型成员变量
  MPC mpc;
  rclcpp::Time robot_time;  // 确保使用 rclcpp::Time
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr mpc_traj_pub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr err_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr traj_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr poly_pub;

  std::string world_frame_id;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_theta = 0.0;
  double current_linear_velocity = 0.0;
  double current_angular_velocity = 0.0;
  double current_curvature = 0.0;
  double current_angle = 0.0;
  double cmd_vel = 0;
  double cmd_acc = 0;
  double cmd_steer_angle = 0;
  double cmd_steer_rate = 0;
  std::vector<double> mpc_x, mpc_y;

  std::vector<tf2::Vector3> control_points;
  double control_points_dl = 2.0;
  std::size_t control_points_num = 6;
  std::vector<double> control_coefs;

  tf2::Transform robot2world;

  using Trajectory = std::list<TrajPtr>;
  Trajectory trajectory;
  Trajectory::iterator current_segment;
  double current_segment_length = 0.0;

  void on_timer();
  void on_pose(const nav_msgs::msg::Odometry::SharedPtr odom);
  /*
   *@brief calculates feedback error for trajectory
   *@return feedback error
   */
  double cross_track_error();

  /// \ returns iterator to segment and current length  of trajectory belonging to current position
  void get_segment(std::list<TrajPtr>::iterator& traj_it, double& len);

  /// \ update robot pose to current time based on last pose and velocities
  void update_robot_pose(double dt);
  /*
   * \brief publishes trajectory as pointcloud message
   */
  void publish_trajectory();
  void on_odo(const nav_msgs::msg::Odometry::SharedPtr odom);
  void update_trajectory_segment();
  void publish_error(double error);

  void update_control_points();
  // converts control points into robot coordinate frame
  void convert_control_points();
  void calculate_control_coefs();
  double polyeval(double x);
  void apply_control();
  void publish_poly();
  void publish_mpc_traj(std::vector<double>& x, std::vector<double>& y);
  tf2::Quaternion createQuaternionMsgFromYaw(
      const double yaw) const;
  void solveMPC();

 public:
  void reset();
  MPCController(const std::string& ns = "mpc_controller");
  virtual ~MPCController();

private:
  // Remove duplicated declarations
  // Remove the following section since these are declared above
  /* 
  int mpc_steps;
  double mpc_dt;
  double max_acc;  
  MPC mpc{...};
  */
};

} /* namespace mpc_controller */

#endif /* MPC_CONTROLLER_H_ */

