#include "self_drive.hpp"

SelfDrive::SelfDrive() : rclcpp::Node("self_drive"), step_(0)
{
  auto lidar_qos = rclcpp::SensorDataQoS();
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", lidar_qos, std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1));

  vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", rclcpp::QoS(10));
}

SelfDrive::~SelfDrive() {}

// 전체 흐름을 제어 
void SelfDrive::subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  ScanData data = preprocess_lidar(scan);
  geometry_msgs::msg::Twist vel = calculate_velocity(data);
  
  publish_velocity(vel);
  
  // 상태 로깅 
  if (step_ % 10 == 0) {
    RCLCPP_INFO(get_logger(), "Step=%d | F:%.2f L:%.2f R:%.2f | Vel:%.2f", 
                step_, data.front, data.left, data.right, vel.linear.x);
  }
  step_++;
}

// 데이터 전처리
ScanData SelfDrive::preprocess_lidar(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  ScanData data;
  data.front = get_avg_range(scan, -10, 10);
  data.left  = get_avg_range(scan, 40, 80);
  data.right = get_avg_range(scan, -80, -40);
  return data;
}

// 거리 평균 계산
float SelfDrive::get_avg_range(const sensor_msgs::msg::LaserScan::SharedPtr scan, int start, int end)
{
  float sum = 0.0f;
  int count = 0;
  size_t size = scan->ranges.size();

  for (int i = start; i <= end; ++i) {
    int idx = (i + 360) % 360;
    if (idx >= (int)size) continue;
    
    float r = scan->ranges[idx];
    if (!std::isfinite(r) || r <= 0.0f) r = 2.0f;
    if (r > 2.0f) r = 2.0f;

    sum += r;
    count++;
  }
  return (count > 0) ? (sum / count) : 2.0f;
}

// 핵심 주행 알고리즘 
geometry_msgs::msg::Twist SelfDrive::calculate_velocity(const ScanData& data)
{
  geometry_msgs::msg::Twist vel;
  float error = data.left - data.right;
  float k_p = (data.front < 0.35f) ? 4.0f : 2.5f; // 코너링 강화 로직

  // 기본 주행
  vel.linear.x = 0.16f;
  vel.angular.z = error * k_p;

  // 비상 회피 (너무 가까울 때만 예외 처리)
  if (data.front < 0.15f) {
    vel.linear.x = 0.05f;
    vel.angular.z = (data.left > data.right) ? 1.5f : -1.5f;
  }

  return vel;
}

// 메시지 발행
void SelfDrive::publish_velocity(const geometry_msgs::msg::Twist& twist)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.twist = twist;

  vel_pub_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
