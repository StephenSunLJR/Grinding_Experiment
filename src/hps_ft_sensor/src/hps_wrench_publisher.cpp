#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

#include "hps_ft_sensor/hps_sensor.h"

namespace hps_ft_sensor {
namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;

bool readVector(ros::NodeHandle& node_handle, const std::string& name,
                std::size_t expected_size, std::vector<double>* value) {
  return node_handle.getParam(name, *value) &&
         value->size() == expected_size;
}

Eigen::Matrix3d rotationFromRpy(const std::vector<double>& rpy) {
  return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

}  // namespace

class HpsWrenchPublisher {
 public:
  HpsWrenchPublisher() : private_node_handle_("~") {}

  bool initialize() {
    private_node_handle_.param("sensor_ip", sensor_ip_, sensor_ip_);
    private_node_handle_.param("sensor_port", sensor_port_, sensor_port_);
    private_node_handle_.param("publish_rate", publish_rate_, publish_rate_);
    private_node_handle_.param("output_topic", output_topic_, output_topic_);
    private_node_handle_.param("output_frame", output_frame_, output_frame_);
    private_node_handle_.param("zero_on_start", zero_on_start_, zero_on_start_);
    private_node_handle_.param("configure_sensor", configure_sensor_,
                               configure_sensor_);
    private_node_handle_.param("filter_alpha", filter_alpha_, filter_alpha_);

    std::vector<double> translation, rpy, bias, scale, deadband;
    if (!readVector(private_node_handle_, "ee_to_sensor_translation", 3,
                    &translation) ||
        !readVector(private_node_handle_, "ee_to_sensor_rpy", 3, &rpy) ||
        !readVector(private_node_handle_, "raw_wrench_bias", 6, &bias) ||
        !readVector(private_node_handle_, "raw_wrench_scale", 6, &scale) ||
        !readVector(private_node_handle_, "output_deadband", 6, &deadband)) {
      ROS_ERROR("HPS F/T sensor: invalid vector parameter size.");
      return false;
    }
    if (sensor_port_ <= 0 || sensor_port_ > 65535 || publish_rate_ <= 0.0 ||
        filter_alpha_ <= 0.0 || filter_alpha_ > 1.0 || output_frame_.empty() ||
        output_topic_.empty()) {
      ROS_ERROR("HPS F/T sensor: invalid scalar or string parameter.");
      return false;
    }

    ee_to_sensor_translation_ =
        Eigen::Map<const Eigen::Vector3d>(translation.data());
    ee_to_sensor_rotation_ = rotationFromRpy(rpy);
    raw_wrench_bias_ = Eigen::Map<const Vector6d>(bias.data());
    raw_wrench_scale_ = Eigen::Map<const Vector6d>(scale.data());
    output_deadband_ = Eigen::Map<const Vector6d>(deadband.data());
    if (!ee_to_sensor_translation_.allFinite() ||
        !ee_to_sensor_rotation_.allFinite() ||
        !raw_wrench_bias_.allFinite() || !raw_wrench_scale_.allFinite() ||
        !output_deadband_.allFinite()) {
      ROS_ERROR("HPS F/T sensor: parameters contain NaN or Inf.");
      return false;
    }

    if (!sensor_.connect_sensor(static_cast<unsigned short>(sensor_port_),
                                sensor_ip_.c_str(), zero_on_start_)) {
      ROS_ERROR_STREAM("HPS F/T sensor: cannot connect to " << sensor_ip_
                                                             << ':'
                                                             << sensor_port_);
      return false;
    }
    if (configure_sensor_ && !sensor_.config_ftc_control()) {
      ROS_ERROR("HPS F/T sensor: vendor configuration failed.");
      return false;
    }

    publisher_ = node_handle_.advertise<geometry_msgs::WrenchStamped>(
        output_topic_, 1, false);
    ROS_INFO_STREAM("HPS F/T sensor: publishing " << output_topic_ << " in "
                                                   << output_frame_ << " at "
                                                   << publish_rate_ << " Hz");
    return true;
  }

  void run() {
    ros::Rate rate(publish_rate_);
    while (ros::ok()) {
      publishOnce();
      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  void publishOnce() {
    double raw_data[6]{};
    if (sensor_.get_ftData(raw_data) != HPS_FTC_SUCCESS) {
      ROS_WARN_THROTTLE(1.0, "HPS F/T sensor: read failed; sample dropped.");
      return;
    }

    Vector6d raw = Eigen::Map<const Vector6d>(raw_data);
    raw = (raw - raw_wrench_bias_).cwiseProduct(raw_wrench_scale_);

    // Transform a wrench from the sensor origin S to the end-effector origin E:
    // F_E = R_E_S F_S, tau_E = R_E_S tau_S + p_E_S x F_E.
    Vector6d transformed;
    transformed.head<3>() = ee_to_sensor_rotation_ * raw.head<3>();
    transformed.tail<3>() =
        ee_to_sensor_rotation_ * raw.tail<3>() +
        ee_to_sensor_translation_.cross(transformed.head<3>());

    if (!filter_initialized_) {
      filtered_wrench_ = transformed;
      filter_initialized_ = true;
    } else {
      filtered_wrench_ = filter_alpha_ * transformed +
                         (1.0 - filter_alpha_) * filtered_wrench_;
    }
    for (Eigen::Index i = 0; i < filtered_wrench_.size(); ++i) {
      if (std::abs(filtered_wrench_[i]) < std::abs(output_deadband_[i])) {
        filtered_wrench_[i] = 0.0;
      }
    }
    if (!filtered_wrench_.allFinite()) {
      ROS_WARN_THROTTLE(1.0, "HPS F/T sensor: invalid sample dropped.");
      return;
    }

    geometry_msgs::WrenchStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = output_frame_;
    message.wrench.force.x = filtered_wrench_[0];
    message.wrench.force.y = filtered_wrench_[1];
    message.wrench.force.z = filtered_wrench_[2];
    message.wrench.torque.x = filtered_wrench_[3];
    message.wrench.torque.y = filtered_wrench_[4];
    message.wrench.torque.z = filtered_wrench_[5];
    publisher_.publish(message);
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher publisher_;
  hps_sensor sensor_;

  std::string sensor_ip_{"172.16.0.3"};
  int sensor_port_{8080};
  double publish_rate_{250.0};
  std::string output_topic_{"/ft_sensor/wrench"};
  std::string output_frame_{"panda_EE"};
  bool zero_on_start_{true};
  bool configure_sensor_{false};
  double filter_alpha_{0.1};

  Eigen::Vector3d ee_to_sensor_translation_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d ee_to_sensor_rotation_{Eigen::Matrix3d::Identity()};
  Vector6d raw_wrench_bias_{Vector6d::Zero()};
  Vector6d raw_wrench_scale_{Vector6d::Ones()};
  Vector6d output_deadband_{Vector6d::Zero()};
  Vector6d filtered_wrench_{Vector6d::Zero()};
  bool filter_initialized_{false};
};

}  // namespace hps_ft_sensor

int main(int argc, char** argv) {
  ros::init(argc, argv, "hps_wrench_publisher");
  hps_ft_sensor::HpsWrenchPublisher publisher;
  if (!publisher.initialize()) {
    return 1;
  }
  publisher.run();
  return 0;
}
