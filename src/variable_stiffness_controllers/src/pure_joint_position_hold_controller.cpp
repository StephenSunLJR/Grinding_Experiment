#include "pure_joint_position_hold_controller.h"

#include <cmath>
#include <exception>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace variable_stiffness_controllers {

bool PureJointPositionHoldController::init(
    hardware_interface::RobotHW* robot_hardware,
    ros::NodeHandle& node_handle) {
  if (robot_hardware == nullptr) {
    ROS_ERROR("PureJointPositionHoldController: RobotHW is null.");
    return false;
  }

  auto* position_interface =
      robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_interface == nullptr) {
    ROS_ERROR("PureJointPositionHoldController: position interface unavailable.");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) ||
      joint_names.size() != 7) {
    ROS_ERROR("PureJointPositionHoldController: joint_names must contain 7 names.");
    return false;
  }

  joint_handles_.reserve(joint_names.size());
  try {
    for (const auto& joint_name : joint_names) {
      joint_handles_.push_back(position_interface->getHandle(joint_name));
    }
  } catch (const hardware_interface::HardwareInterfaceException& exception) {
    ROS_ERROR_STREAM("PureJointPositionHoldController: handle error: "
                     << exception.what());
    return false;
  }
  return true;
}

void PureJointPositionHoldController::starting(const ros::Time& /*time*/) {
  hold_position_valid_ = true;
  for (std::size_t i = 0; i < hold_position_.size(); ++i) {
    hold_position_[i] = joint_handles_[i].getPosition();
    if (!std::isfinite(hold_position_[i])) {
      hold_position_valid_ = false;
      ROS_FATAL("PureJointPositionHoldController: non-finite start position; commands disabled.");
    }
  }
  if (!hold_position_valid_) {
    return;
  }
  ROS_INFO_STREAM("PureJointPositionHoldController: holding q = ["
                  << hold_position_[0] << ", " << hold_position_[1] << ", "
                  << hold_position_[2] << ", " << hold_position_[3] << ", "
                  << hold_position_[4] << ", " << hold_position_[5] << ", "
                  << hold_position_[6] << "]");
}

void PureJointPositionHoldController::update(
    const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  if (!hold_position_valid_) {
    return;
  }
  for (std::size_t i = 0; i < hold_position_.size(); ++i) {
    joint_handles_[i].setCommand(hold_position_[i]);
  }
}

}  // namespace variable_stiffness_controllers

PLUGINLIB_EXPORT_CLASS(
    variable_stiffness_controllers::PureJointPositionHoldController,
    controller_interface::ControllerBase)
