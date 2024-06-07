#ifndef MPU9250DRIVER_H
#define MPU9250DRIVER_H

#include "mpu9250sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MPU9250Driver : public rclcpp::Node {
 public:
  MPU9250Driver();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
  std::unique_ptr<MPU9250Sensor> mpu9250_;
  size_t count_;
  rclcpp::Time stamp_;
  int moving_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  void handleInput();
  void readVel(const geometry_msgs::msg::Twist::SharedPtr message);
  void declareParameters();
  void calculateOrientation(sensor_msgs::msg::Imu& imu_message);
};

#endif  // MPU9250DRIVER_H
