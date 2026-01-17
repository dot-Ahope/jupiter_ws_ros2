#ifndef TRANSBOT_BASE_H
#define TRANSBOT_BASE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"

class RobotBase : public rclcpp::Node
{
public:
    RobotBase();

private:
    void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double linear_velocity_x_;
    double linear_velocity_y_;
    double angular_velocity_z_; // From cmd_vel (Target)
    double imu_angular_z_;      // From IMU (Actual)
    bool use_imu_for_angular_;  // Flag to use IMU data

    rclcpp::Time last_vel_time_;
    double vel_dt_;
    double x_pos_;
    double y_pos_;
    double heading_;
    double linear_scale_;
    double angular_scale_;
    bool is_multi_robot_;
    std::string namespace_;
};

#endif // TRANSBOT_BASE_H
