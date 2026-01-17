#include "jupiter_base/base.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

RobotBase::RobotBase()
    : Node("base_node"),
      linear_velocity_x_(0),
      linear_velocity_y_(0),
      angular_velocity_z_(0),
      imu_angular_z_(0.0),
      use_imu_for_angular_(false),
      vel_dt_(0),
      x_pos_(0),
      y_pos_(0),
      heading_(0)
{
    // Declare parameters
    this->declare_parameter("linear_scale", 1.0);
    this->declare_parameter("angular_scale", 1.0);
    this->declare_parameter("is_namespace", "");
    this->declare_parameter("is_multi_robot", false);
    this->declare_parameter("use_imu_for_odom", true);

    // Get parameters
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    namespace_ = this->get_parameter("is_namespace").as_string();
    is_multi_robot_ = this->get_parameter("is_multi_robot").as_bool();
    use_imu_for_angular_ = this->get_parameter("use_imu_for_odom").as_bool();

    // Create publishers and subscribers
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odom_raw", 50);
    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/jupiter/get_vel", 50,
        std::bind(&RobotBase::velCallback, this, std::placeholders::_1));
    
    // Subscribe to calibrated IMU data
    // Use "Best Effort" QoS for sensor data to reduce latency
    auto qos = rclcpp::SensorDataQoS();
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_calibrated", qos,
        std::bind(&RobotBase::imuCallback, this, std::placeholders::_1));

    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_vel_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Base node has been initialized. IMU Fusion: %s", 
        use_imu_for_angular_ ? "Enabled" : "Disabled");
}

void RobotBase::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Store latest IMU yaw rate
    // Note: Direction might need to be checked against angular_scale sign
    imu_angular_z_ = msg->angular_velocity.z;
}

void RobotBase::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    auto current_time = this->now();

    linear_velocity_x_ = msg->linear.x * linear_scale_;
    linear_velocity_y_ = msg->linear.y * linear_scale_;
    
    // Determine which angular velocity to use
    if (use_imu_for_angular_) {
        // Use real sensor data if available
        // angular_scale not applied to IMU data as it should be calibrated already
        angular_velocity_z_ = imu_angular_z_; 
    } else {
        // Fallback to command-based estimation
        angular_velocity_z_ = msg->angular.z * angular_scale_;
    }
    
    vel_dt_ = (current_time - last_vel_time_).seconds();
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_;
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_;
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_;

    // 업데이트된 위치 및 방향 계산
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;


    // Create quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, heading_);

    // Create and publish odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    
    if (!is_multi_robot_) {
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
    } else {
        odom.header.frame_id = namespace_ + "/odom";
        odom.child_frame_id = namespace_ + "/base_footprint";
    }

    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.angular.z = angular_velocity_z_;

    odom_publisher_->publish(odom);

    // TF 발행을 robot_localization에 맡기기 위해 주석 처리
    // EKF가 Odom + IMU를 융합하여 더 정확한 TF 발행
    // Publish transform
    // geometry_msgs::msg::TransformStamped transformStamped;
    // transformStamped.header.stamp = current_time;
    // transformStamped.header.frame_id = odom.header.frame_id;
    // transformStamped.child_frame_id = odom.child_frame_id;
    // transformStamped.transform.translation.x = x_pos_;
    // transformStamped.transform.translation.y = y_pos_;
    // transformStamped.transform.translation.z = 0.0;
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();

    // tf_broadcaster_->sendTransform(transformStamped);
}
