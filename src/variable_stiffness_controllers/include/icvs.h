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
#include "variable_stiffness_controllers/compliance_robot.h"//在devel中，从msg文件中生成
namespace variable_stiffness_controllers {

class icvs : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
 
 private:
  double count;
  ros::Duration t;
  ros::Publisher pub;
  ros::Subscriber sub;
  variable_stiffness_controllers::compliance_robot robot;
  void  doEmg(const std_msgs::String::ConstPtr& msg_p);
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  
  
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;  //机械臂模型节点
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  //机械臂状态节点
  std::vector<hardware_interface::JointHandle> joint_handles_;  //机械臂关节节点
  static constexpr double kDeltaTauMax{1.0}; //控制力矩变化的
  
  double radius_{0.15};
  double factor_circle{0.2};
  double a_circle{0};
  double b_circle{0};
  double c_circle{0};

  std::vector<double>  md_gains_;   //M_d
  std::vector<double>  dd_gains_;  //D_d
  std::vector<double>  kd_gains_;   //K_d
  std::array<double, 7>  q_init_array;
  std::array<double, 7>  dq_init_array;
  std::array<double, 7>  ddq_init_array;

  std::array<double, 7>  q_array;  //实际位置
  std::array<double, 7>  dq_array; //实际速度
  std::array<double, 7>  ddq_array;//实际加速度

  std::array<double, 7>  qd_array; //期望位置
  std::array<double, 7>  dqd_array; //期望速度
  std::array<double, 7>  ddqd_array; //期望家速度

  Eigen::Matrix<double, 7, 1>   tau_cmd; //期望力矩
  std::array<double, 7>  tau_ext_array;
  std::array<double, 7>  tau_J_d_array; 
  //模型信息
  std::array<double, 49> mass_array;
  std::array<double, 7>  coriolis_array;
  std::array<double, 7>  gravity_array;

  std::array<double, 42> jacobian_old_array;
  Eigen::Matrix<double, 6, 6>   I;
public:
  double xt_;
  Eigen::Matrix<double, 6, 1> wt_;
  double kt_{0.0};
  double last_kt;
  double kc_;
  double delta{0.0};
  double limit_max{0.0};
  double limit_min{0.0};
  bool upflag{true};
  double kt_max;
  double kt_min;
  static constexpr double DeltaKMax{1.0};
  double min_tank_level_;
  double max_tank_level_;
  double sigma_;
  bool firstUpdate{true};
  
  Eigen::Matrix<double,6,7> S1;
  Eigen::Matrix<double,6,7> S1_dot;
  

  Eigen::Matrix<double, 6,1> f_tau;
  //md dd kd
  Eigen::DiagonalMatrix<double, 6,6> md;
  Eigen::DiagonalMatrix<double, 6,6> dd;
  Eigen::DiagonalMatrix<double, 6,6> kd;
  
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Matrix<double,6,1>velocity_d_;
  Eigen::Matrix<double,6,1>error_velocity;
  Eigen::Matrix<double,6,1>dvelocity_d_;
  Eigen::Quaterniond angular_velocity_d_;
  std::array<double, 3>  f_x_ctotal{0}; //期望位置
};

}  
