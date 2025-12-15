#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;
  int step_;

public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0)
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    // std::bind 사용 시 클래스 멤버 함수 바인딩
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", lidar_qos_profile, callback);

    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", vel_qos_profile);
  }

  float get_avg_range(const sensor_msgs::msg::LaserScan::SharedPtr scan, int start_deg, int end_deg)
  {
    float sum = 0.0f;
    int count = 0;
    int size = scan->ranges.size();

    for (int i = start_deg; i <= end_deg; ++i)
    {

      int idx = (i + 360) % 360; 
      if (idx >= size) continue;
      
      float r = scan->ranges[idx];
      
      if (!std::isfinite(r) || r <= 0.0f) {
        r = 2.0f; 
      }

      if (r > 2.0f) r = 2.0f;

      sum += r;
      count++;
    }
    
    return (count > 0) ? (sum / count) : 2.0f;
  }


  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::Twist vel;
    
    // 정면 (전방 충돌 감지용)
    float front = get_avg_range(scan, -10, 10);
    // 좌측 대각
    float left  = get_avg_range(scan, 40, 80);
    // 우측 대각
    float right = get_avg_range(scan, -80, -40);

    float base_speed = 0.16f; 
    
    float error = left - right;

    float k_p = 2.5f; 
    
    // 전방에 벽이 가까워지면 회전력을 더 강하게 줌 (코너링)
    if (front < 0.35f) {
        // 정면이 막혔을 때는 error에 가중치를 더 주거나, 
        // 전진 속도를 아주 살짝 줄여서(그래도 0.15 유지 노력) 회전 반경 확보
        k_p = 4.0f; 
    }
    vel.linear.x = base_speed;
    vel.angular.z = error * k_p;
    // 로봇 지름이 0.25m이므로 벽까지 0.125m 이하면 충돌. 
    // 여유 있게 0.15m 미만이면 후진보다는 제자리 회전 시도(하지만 전진 속도 유지 요구사항 우선)
    if (front < 0.15f) {
         // 충돌 직전에는 어쩔 수 없이 감속해야 벽을 안 긁음.
         // 하지만 요구사항상 최대한 전진하려 노력. 회전 극대화.
         vel.linear.x = 0.05f; 
         // 더 넓은 쪽으로 강제 회전
         vel.angular.z = (left > right) ? 1.5f : -1.5f; 
    }

    RCLCPP_INFO(rclcpp::get_logger("self_drive"),
                "Step=%d | F:%.2f L:%.2f R:%.2f | Err:%.2f | Vel:%.2f Ang:%.2f", 
                step_, front, left, right, error, vel.linear.x, vel.angular.z);

    pose_pub_->publish(vel);
    step_++;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

