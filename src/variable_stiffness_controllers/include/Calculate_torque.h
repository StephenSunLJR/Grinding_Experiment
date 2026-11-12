#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "variable_stiffness_controllers/compliance_robot.h"

namespace variable_stiffness_controllers {

class Calculate_torque : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  private:
  variable_stiffness_controllers::compliance_robot robot;
  double count;
  ros::Duration t;
  ros::Publisher pub;
  const double delta_tau_max_{1.0};
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  
  
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;  //机械臂模型节点
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  //机械臂状态节点
  std::vector<hardware_interface::JointHandle> joint_handles_;  //机械臂关节节点
  static constexpr double kDeltaTauMax{1.0}; //控制力矩变化的
  Eigen::Matrix<double, 7, 1> q_initial;
  Eigen::Matrix<double,7,1> q_desire;
  Eigen::Matrix<double ,7,1> dq_desire;
  Eigen::Matrix<double ,7,1> ddq_desire;

  Eigen::Matrix<double ,7,7> Kd;
  Eigen::Matrix<double ,7,7 > Kp;
  
  Eigen::Matrix<double ,7,1 >error;
  Eigen::Matrix<double ,7,1 > derror;
  Eigen::Matrix<double ,7,1 > tau;

};
}