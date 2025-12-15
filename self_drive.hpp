#ifndef SELF_DRIVE_HPP_
#define SELF_DRIVE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

// 데이터 전달을 위한 구조체
struct ScanData {
  float front;
  float left;
  float right;
};

class SelfDrive : public rclcpp::Node
{
public:
  SelfDrive();
  ~SelfDrive();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  int step_;

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  // 라이다로 필요한 방향의 거리 측정
  ScanData preprocess_lidar(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  // 특정 각도 범위의 평균 거리 계산
  float get_avg_range(const sensor_msgs::msg::LaserScan::SharedPtr scan, int start_deg, int end_deg);

  //센서의 데이터로 속도와 회전각 계산
  geometry_msgs::msg::Twist calculate_velocity(const ScanData& data);

  //계산된 속도를 ROS 메시지로 표시
  void publish_velocity(const geometry_msgs::msg::Twist& twist);
};

#endif  // SELF_DRIVE_HPP_

