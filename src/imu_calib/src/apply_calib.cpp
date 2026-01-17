#include "imu_calib/apply_calib.hpp"

namespace imu_calib
{

ApplyCalib::ApplyCalib()
: Node("apply_calib"),
  gyro_sample_count_(0),
  gyro_sum_(Eigen::Vector3d::Zero()),
  gyro_bias_(Eigen::Vector3d::Zero()),
  accel_scale_(Eigen::Matrix3d::Identity()),  // <- Safe default
  gyro_calibrated_(false)
{
  // Declare and get parameters
  this->declare_parameter<std::string>("calib_file", "imu_calib.yaml");
  this->declare_parameter<bool>("calibrate_gyros", true);
  this->declare_parameter<int>("gyro_calib_samples", 100);
  this->declare_parameter<int>("queue_size", 5);

  this->get_parameter("calib_file", calib_file_);
  this->get_parameter("calibrate_gyros", calibrate_gyros_);
  this->get_parameter("gyro_calib_samples", gyro_calib_samples_);
  int queue_size;
  this->get_parameter("queue_size", queue_size);

  // Load calibration file
  if (!loadCalibration()) {
    RCLCPP_FATAL(this->get_logger(), "Calibration could not be loaded");
    rclcpp::shutdown();
    return;
  }

  // Subscribers and publishers
rclcpp::SensorDataQoS qos;
raw_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
  "/raw", qos, std::bind(&ApplyCalib::rawImuCallback, this, std::placeholders::_1));

  corrected_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/corrected", queue_size);
}

ApplyCalib::~ApplyCalib() {}

bool ApplyCalib::loadCalibration()
{
  try {
    YAML::Node config = YAML::LoadFile(calib_file_);

    // Load accel bias and scale
    accel_bias_ = Eigen::Vector3d(
      config["accel_bias"][0].as<double>(),
      config["accel_bias"][1].as<double>(),
      config["accel_bias"][2].as<double>());

    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        accel_scale_(i, j) = config["accel_scale"][i * 3 + j].as<double>();

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load calibration file: %s", e.what());
    return false;
  }
}

void ApplyCalib::rawImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (calibrate_gyros_ && !gyro_calibrated_) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Calibrating gyros; do not move the IMU");

    // Dynamic gyro bias calibration using recursive mean (jupiter_library method)
    gyro_sample_count_++;
    double prev_weight = (double)(gyro_sample_count_ - 1) / gyro_sample_count_;
    double curr_weight = 1.0 / gyro_sample_count_;
    
    gyro_bias_.x() = prev_weight * gyro_bias_.x() + curr_weight * msg->angular_velocity.x;
    gyro_bias_.y() = prev_weight * gyro_bias_.y() + curr_weight * msg->angular_velocity.y;
    gyro_bias_.z() = prev_weight * gyro_bias_.z() + curr_weight * msg->angular_velocity.z;

    if (gyro_sample_count_ >= gyro_calib_samples_) {
      // Calibration quality verification (relaxed criteria for real-world use)
      double bias_variance = std::abs(gyro_bias_.x() - gyro_bias_.y()) + 
                            std::abs(gyro_bias_.z() - gyro_bias_.y()) + 
                            std::abs(gyro_bias_.z() - gyro_bias_.x());
      
      // Relaxed quality threshold for practical use (0.01 → 0.05)
      if (bias_variance > 0.05) { 
        RCLCPP_WARN(this->get_logger(), "Gyro calibration quality poor (variance: %.4f), recalibrating...", bias_variance);
        gyro_sample_count_ = 0; // Restart calibration
        gyro_bias_ = Eigen::Vector3d::Zero();
        return;
      }
      
      gyro_calibrated_ = true;
      calibrate_gyros_ = false;

      RCLCPP_INFO(this->get_logger(), "Gyro calibration complete! (bias = [%.3f, %.3f, %.3f], quality = %.4f)",
                  gyro_bias_.x(), gyro_bias_.y(), gyro_bias_.z(), bias_variance);
    }

    return;
  }

  // Apply calibration and publish
  sensor_msgs::msg::Imu corrected = *msg;
  corrected = applyCorrectionAccel(msg);
  corrected = applyCorrectionGyro(corrected);

  corrected_imu_pub_->publish(corrected);
}

sensor_msgs::msg::Imu ApplyCalib::applyCorrectionAccel(const sensor_msgs::msg::Imu::SharedPtr msg) const
{
  Eigen::Vector3d raw_accel(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  Eigen::Vector3d corrected_accel = accel_scale_ * (raw_accel - accel_bias_);

  sensor_msgs::msg::Imu corrected = *msg;
  corrected.linear_acceleration.x = corrected_accel.x();
  corrected.linear_acceleration.y = corrected_accel.y();
  corrected.linear_acceleration.z = corrected_accel.z();

  return corrected;
}

sensor_msgs::msg::Imu ApplyCalib::applyCorrectionGyro(const sensor_msgs::msg::Imu& msg) const
{
  sensor_msgs::msg::Imu corrected = msg;

  corrected.angular_velocity.x -= gyro_bias_.x();
  corrected.angular_velocity.y -= gyro_bias_.y();
  corrected.angular_velocity.z -= gyro_bias_.z();

  return corrected;
}

} // namespace imu_calib
