#pragma once 
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl/tree.hpp>

//引入ROS标准的KDL运动学库

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include "variable_stiffness_controllers/compliance_robot.h"
#include "../include/franka_utils/pseudo_inversion.h"
#include "../include/franka_utils/Dynamics.h"

namespace variable_stiffness_controllers {

class IcvsEnergyMujoco : public controller_interface::MultiInterfaceController<
                                    hardware_interface::EffortJointInterface,
                                    hardware_interface::JointStateInterface> {
public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void doEmg(const std_msgs::String::ConstPtr& msg_p);
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& last_tau_cmd);

  // 通用硬件接口
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // KDL 运动学求解器
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  // ROS 通信
  ros::Publisher pub;
  ros::Subscriber sub;
  compliance_robot robot;

  // 状态与控制变量
  Eigen::Matrix<double, 7, 1> last_tau_cmd_; // 替代原先的 tau_J_d
  bool firstUpdate = true;
  double count = 0.0;
  ros::Duration t;
  
  // 原有的所有控制算法参数
  double sigma_, xt_, kc_, kt_, kt_max, kt_min, min_tank_level_, max_tank_level_, lambda, last_kt;
  double dela_xt;
  
  std::array<double, 7> q_init_array, dq_init_array, ddq_init_array;
  std::array<double, 7> qd_array, dqd_array, ddqd_array;
  std::array<double, 7> q_array, dq_array;
  
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Matrix<double, 6, 1> velocity_d_, dvelocity_d_, error_velocity;
  
  Eigen::Matrix<double, 6, 6> md, dd, kc_matrix;
  Eigen::Matrix<double, 6, 6> kt_matrix;
  Eigen::Matrix<double, 6, 6> I, r;
  
  double limit_max, limit_min;
  double a_circle, b_circle, c_circle;
  double a = 1.0; 
  
  Eigen::Matrix<double, 6, 1> f_tau;
  Eigen::Matrix<double, 6, 7> S1, S1_dot;
  Eigen::Matrix<double, 6, 7> jacobian_old;
  
  Eigen::Matrix<double, 6, 1> wt_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
  
  const double kDeltaTauMax = 1.0;
};

}