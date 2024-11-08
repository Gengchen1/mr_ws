#ifndef SRC_TRAJECTORY_SEGMENT_H_
#define SRC_TRAJECTORY_SEGMENT_H_

#include <tf2/LinearMath/Vector3.h>
#include <angles/angles.h>

namespace trajectory {
/* 
 * 曲率恒定的轨迹部分
 * x0 - 起点
 * v - 起始（单位）矢量
 * length - 轨迹段长度
 * 基类
 */
class TrajectorySegment {
public:
  // 返回轨迹的长度
  virtual double get_length() const = 0;
  // 返回曲线的曲率
  virtual double get_curvature() const = 0;
  // 返回给定长度的轨迹点(使用三维向量表示)
  virtual tf2::Vector3 get_point(double point_len) const = 0;
  // 返回给定长度处轨迹点的朝向角度
  virtual double get_orientation(double point_len) const = 0;
  // 返回给定点距离最近的轨迹点的长度(角度除以曲率(单位长度下的角度变化量)得到轨迹长度)
  virtual double get_point_length(double x, double y) const = 0;
  // 返回给定点距离最近的轨迹点 w r t 方向的距离(使用旋转矩阵计算)
  virtual double get_point_distance(double x, double y) const = 0;

  virtual ~TrajectorySegment() {}
};

// 圆轨迹
class CircularSegment : public TrajectorySegment {
// 可以被派生类访问
protected:
  // 曲率 1/R
  double curvature;
  // 起始点
  tf2::Vector3 x0;
  // 起始点出的切向量
  tf2::Vector3 v;
  // 轨迹长度
  double length;
  // 从圆心到第一点的角度
  double start_angle;
  // 从圆心到最后一个点的角度
  double finish_angle;
  // 圆心坐标
  tf2::Vector3 center;

public:
  double get_length() const { return length; }
  
  double get_curvature() const { return curvature; }
  //生成从轨迹起始点到指定轨迹长度的点的坐标
  // 这个点的坐标由圆弧的起始点、曲率和沿圆弧的长度来定义
  tf2::Vector3 get_point(double point_len) const {
    // 圆弧上某点到圆心的(单位)向量，v是该点垂直于圆弧的切线向量
    tf2::Vector3 toCenter(-v.y(), v.x(), 0);
    // 计算圆心的位置向量：起始向量加上(指向圆心的单位向量×R)
    tf2::Vector3 center = x0 + toCenter / curvature;
    // 计算从圆弧起始点到目标点的角度angle。
    // 这里start_angle是圆弧起始点的角度, 
    // point_len是沿圆弧的长度，curvature是圆弧的曲率。
    double angle = start_angle + point_len * curvature;
    // 返回目标点的位置向量, 圆心位置向量加上
    return center + tf2::Vector3(cos(angle), sin(angle), 0) / fabs(curvature);
  }
  // 返回给定长度处轨迹点的朝向角度
  double get_orientation(double point_len) const {
    // 起始角度加上转过的角度即指定点在以圆心为原点的坐标系上的角度，
    // 其和朝向(圆弧切线)成90°，如果曲率为负，即顺时针转，再-90°就是切线朝向
    // 反之亦然。
    return start_angle + point_len * curvature + 
    ((curvature > 0.0) ? M_PI/2 : -M_PI/2);
  }

  // 返回给定点的轨迹长度
  double get_point_length(double x, double y) const {
    // 计算从轨迹圆心到给定点向量在以圆心为坐标系上的角度。
    double point_angle = atan2(y - center.y(), x - center.x());
    // 并将其标准化到[−π,π]范围内。
    point_angle = angles::normalize_angle(point_angle - start_angle);
    // 返回轨迹长度，角度除以弧长。
    return point_angle / curvature;
  }
  // 看不懂啊！！！！
  double get_point_distance(double x, double y) const {
    double point_angle = atan2(y - center.y(), x - center.x());
    double point_angle_rel = angles::normalize_angle(point_angle - start_angle);
    tf2::Vector3 vr = v.rotate(tf2::Vector3(0, 0, 1), point_angle_rel);
    tf2::Vector3 delta = tf2::Vector3(x, y, 0) - center- tf2::Vector3(
      cos(point_angle)/fabs(curvature), sin(point_angle)/fabs(curvature), 0);
      return vr.cross(delta).z();
  }

  // 构造函数
  CircularSegment(double c, double x, double y, double vx, double vy, double l)
     : curvature(c), x0(x, y, 0), v(vx, vy, 0), length(l),
    // start_angle( (curvature > 0) ? atan2(-v.x(), v.y()) : atan2(v.x(), -v.y()) ),
    // 从圆心到起始点的向量角度
    start_angle( (curvature > 0) ? atan2(v.y(), -v.x()) : atan2(-v.y(), v.x()) ),
    finish_angle(start_angle + length * curvature),
    center(x0 + tf2::Vector3(-v.y(), v.x(), 0) / curvature)
    { v.normalize(); }
};
class LinearSegment : public TrajectorySegment {
protected:
  tf2::Vector3 x0;
  tf2::Vector3 v;
  double length;

public:
  double get_curvature() const { return 0; }

  double get_length() const { return length; }

  tf2::Vector3 get_point(double point_len) const {
    return x0 + point_len * v;
  }

  double get_orientation(double /*point_len*/) const {
    return atan2(v.y(), v.x());
  }
  
  double get_point_length(double x, double y) const {
    // 向量v和向量(x - x0.x(), y - x0.y()) 直接的点积
    double dx = x - x0.x();
    double dy = y - x0.y();
    return v.x() * dx + v.y() * dy;
  }

  double get_point_distance(double x, double y) const {
    double dx = x - x0.x();
    double dy = y - x0.y();
    tf2::Vector3 p ( dx, dy, 0.0 );
    return v.cross(p).z();
  }

  LinearSegment(double x, double y, double vx, double vy, double l)
     : x0(x, y, 0), v(vx, vy, 0), length(l) {}
};

}

#endif /* SRC_TRAJECTORY_SEGMENT_H_ */


 //