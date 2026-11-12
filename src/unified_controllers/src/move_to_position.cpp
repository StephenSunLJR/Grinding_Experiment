#include "../include/move_to_position.h"

#include <cmath>
#include <algorithm>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace unified_controllers {

bool MoveToPosition::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "MoveToPosition: Error getting position joint interface from hardware!");
    return false;
  }
   if (!node_handle.getParam("factor", factor_)) {
    ROS_INFO_STREAM(
        "MoveToPosition: No parameter factor, defaulting to: " << factor_);
  }
  if (std::fabs(factor_) > 1.0) {
    ROS_INFO_STREAM("MoveToPosition: Set factor too large, defaulting to: " << 1.0);
    factor_ = 1.0;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("MoveToPosition: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("MoveToPosition: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "MoveToPosition: Exception getting joint handles: " << e.what());
      return false;
    }
  }

    for (size_t i = 0; i < 7; i++) {
      dq_max_[i] *= factor_;
      ddq_max_[i] *= factor_;
    }

  return true;
}

void MoveToPosition::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    q_start_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
  std::array<double, 7> dq_max_reach{{0, 0, 0, 0, 0, 0, 0}};
   std::array<double, 7> t_f{{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> t_1{{0, 0, 0, 0, 0, 0, 0}};
    double max_t_f = 0;

   for (size_t i = 0; i < 7; i++) {
     delta_q_[i] = q_goal_[i] - q_start_[i];
     dq_max_reach[i] = dq_max_[i];
       if (delta_q_[i] > 0) {
      sign_delta_q[i] = 1;
    }
    else if (delta_q_[i] < 0)
    {
       sign_delta_q[i] = -1;
    }
    else
    {
        sign_delta_q[i] = 0;
    }
    }  

    for (size_t i = 0; i < 7; i++) {
      if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
        if (std::abs(delta_q_[i]) < (3.0 / 2.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_[i]))) {
          dq_max_reach[i] = std::sqrt(2.0 / 3.0 * delta_q_[i] * sign_delta_q[i] * ddq_max_[i]);
        }
        t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_[i];
        t_f[i] = t_1[i] + std::abs(delta_q_[i]) / dq_max_reach[i];
        if (t_f[i] > max_t_f) {
          max_t_f = t_f[i];
        }
      }
    }
    for (size_t i = 0; i < 7; i++) {
      if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
        double a = 1.5 * ddq_max_[i];
        double b = -1.0 * max_t_f * ddq_max_[i] * ddq_max_[i];
        double c = std::abs(delta_q_[i]) * ddq_max_[i] * ddq_max_[i];
        double delta = b*b - 4.0 * a *c;
        if (delta < 0.0) {
          delta = 0.0;
        }
        dq_max_sync_[i] = (-1.0 * b -std::sqrt(delta)) / (2.0 * a);
        t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_[i];
        t_f_sync_[i] = t_1_sync_[i] + std::abs(delta_q_[i] / dq_max_sync_[i]);
        t_2_sync_[i] = t_f_sync_[i] - t_1_sync_[i];
        q_1_[i] = dq_max_sync_[i] * sign_delta_q[i] * (0.5 * t_1_sync_[i]);
        t_d_[i] = t_2_sync_[i] - t_1_sync_[i];
      }
    }
}

void MoveToPosition::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  std::array<double, 7> delta_q_d{{0, 0, 0, 0, 0, 0, 0}};
   
for (size_t i = 0; i < 7; i++) {
  if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
    if (elapsed_time_.toSec() < t_1_sync_[i]) {
      delta_q_d[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] * (0.5 * elapsed_time_.toSec() - t_1_sync_[i]) * std::pow(elapsed_time_.toSec(), 3.0);
    }
    else if (elapsed_time_.toSec() >= t_1_sync_[i] && elapsed_time_.toSec() < t_2_sync_[i]) {
      delta_q_d[i] = q_1_[i] + (elapsed_time_.toSec() - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
    }
    else if (elapsed_time_.toSec() >= t_2_sync_[i] && elapsed_time_.toSec() < t_f_sync_[i]) {
      delta_q_d[i] = delta_q_[i] + 0.5 *
       (1.0 / std::pow(t_1_sync_[i], 3.0) * 
       (elapsed_time_.toSec() - 3*t_1_sync_[i] - t_d_[i]) *
        std::pow((elapsed_time_.toSec() - t_1_sync_[i] - t_d_[i]), 3.0) +
         (2.0*elapsed_time_.toSec()-3.0*t_1_sync_[i]-2.0*t_d_[i]))*
         dq_max_sync_[i]*sign_delta_q[i];
    }
    else
    {
      delta_q_d[i] = delta_q_[i];
    }
  }
}


  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(q_start_[i] + delta_q_d[i]);
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(unified_controllers::MoveToPosition,
                       controller_interface::ControllerBase)