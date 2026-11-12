#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace variable_stiffness_controllers {

// Holds the seven joint positions captured in starting().  This controller
// deliberately has no force/torque subscriber, model interface, state
// interface, or q/dq feedback calculation.  Position tracking below this
// interface is performed by Franka's internal joint-position control mode.
class PureJointPositionHoldController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::array<double, 7> hold_position_{};
  bool hold_position_valid_{false};
};

}  // namespace variable_stiffness_controllers
