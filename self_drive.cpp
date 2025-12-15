#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>
// Twist 대신 TwistStamped 사용
#include <geometry_msgs/msg/twist_stamped.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // Publisher 타입을 TwistStamped로 변경
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_pub_;
  int step_;

public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0)
  {
    auto lidar_qos_profile = rclcpp::SensorDataQoS();
    
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", lidar_qos_profile, callback);

    auto vel_qos_profile = rclcpp::QoS(10);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", vel_qos_profile);
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
      
      if (!std::isfinite(r) || r <= 0.0f) r = 2.0f; 
      if (r > 2.0f) r = 2.0f;

      sum += r;
      count++;
    }
    return (count > 0) ? (sum / count) : 2.0f;
  }

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::Twist vel;


    float front = get_avg_range(scan, -10, 10);
    float left  = get_avg_range(scan, 40, 80);
    float right = get_avg_range(scan, -80, -40);


    float base_speed = 0.16f; 
    float error = left - right;
    float k_p = 2.5f; 


    if (front < 0.35f) {
        k_p = 4.0f; 
    }

    vel.linear.x = base_speed;
    vel.angular.z = error * k_p;


    if (front < 0.15f) {
         vel.linear.x = 0.05f; 
         vel.angular.z = (left > right) ? 1.5f : -1.5f; 
    }


    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link"; 
    msg.twist = vel; 

    RCLCPP_INFO(rclcpp::get_logger("self_drive"),
                "Step=%d | F:%.2f L:%.2f R:%.2f | Err:%.2f | Vel:%.2f", 
                step_, front, left, right, error, vel.linear.x);

    pose_pub_->publish(msg); 
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

