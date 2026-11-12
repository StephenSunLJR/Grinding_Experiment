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
#include "unified_controllers/unified_robot.h"
#include <fstream>
namespace unified_controllers {

class UnifiedController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
 
 public:
  double count;
  ros::Duration t;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;  //机械臂模型节点
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  //机械臂状态节点
  std::vector<hardware_interface::JointHandle> joint_handles_;  //机械臂关节节点
  static constexpr double kDeltaTauMax{1.0}; //控制力矩变化的
  std::vector<double>  kd_gains1_;
  std::vector<double>  d_gains1_;
  Eigen::Matrix<double, 6,6> ki;
  Eigen::Matrix<double, 6,6> ki_target;
  Eigen::Matrix<double, 6,6> kp;
  Eigen::Matrix<double, 6,6> kp_target;
  Eigen::Matrix<double, 6,6> kd;
  Eigen::Matrix<double, 6,6> kd_target;
  Eigen::Matrix<double, 6,6> Damping;
  Eigen::Matrix<double, 6,6> Stiffiness;
  Eigen::Matrix<double, 6,6> Damping_end;
  Eigen::Matrix<double, 6,6> Stiffiness_end;
 private:
  std::array<double, 7>  q_init_array;
  std::array<double, 7>  dq_init_array;
  std::array<double, 7>  ddq_init_array;
  std::array<double, 7>  q_array;  //实际位置
  std::array<double, 7>  dq_array; //实际速度
  std::array<double, 7>  ddq_array;//实际加速度
  std::array<double, 7>  qd_array; //期望位置
  std::array<double, 7>  dqd_array; //期望速度
  std::array<double, 7>  ddqd_array; //期望加速度
  std::array<double, 7>  tau_J_d_array; //实际力矩
  std::array<double, 7>  i_err_array;
  std::array<double, 7>  last_dq_array;
  Eigen::Matrix<double, 7, 1> last_dq;
  Eigen::Matrix<double, 7, 1> joint_err;

  bool firstUpdate{true};
  std::array<double, 42> jacobian_old_array;
  Eigen::Matrix<double,6,7> S1;
  Eigen::Matrix<double,6,7> S1_dot;
  double f_end_target;

  double a_circle;
  double b_circle;
  double c_circle;
  double radius{0.15};
  double factor_circle{0.1};
  bool flag{true};
  double a1 {0.1988};
  double b1 {5.95};
  double c1 {1.871};
  double a2 {0.1032};
  double b2 {11.83};
  double c2 {-0.8545};
  double a3 {0.04102};
  double b3 {14.55};
  double c3 {2.514};
//   double a1 {0.2127};
//   double b1 {6.004};
//   double c1 {1.944};
//   double a2 {0.6151};
//   double b2 {11.82};
//   double c2 {-0.6084};
//   double a3 {0.5459};
//   double b3 {12.08};
//   double c3 {2.572};
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d d_position;
  Eigen::Vector3d d_orientation;
  
  Eigen::Matrix<double, 7, 1>  tau_cmd; //期望力矩
  std::array<double, 49> mass_array;
  std::array<double, 7>  coriolis_array;
  std::array<double, 7>  gravity_array;
  unified_controllers::unified_robot robot;

  double xt;
  Eigen::Matrix<double,1, 1> dxt;
  double alpha{1.0};
  double beta{1.0};
  double gamma{0.5};
  double rho_frc{1.0};
  double rho_imp{0.0};
  Eigen::Matrix<double, 6, 1> omega;
  Eigen::Matrix<double, 6, 1> f_end;  //末端执行器坐标下期望力
  Eigen::Matrix<double, 6, 1> fd;
  Eigen::Matrix<double, 6, 1>f_end_last;
  Eigen::Matrix<double, 6, 1>dfd;
  Eigen::Matrix<double, 6, 1>fe_last;
  Eigen::Matrix<double, 6, 1>dfe;
  Eigen::Matrix<double, 6, 1>fi;
  Eigen::Matrix<double, 6, 1>f_init;
  Eigen::Matrix<double, 6, 1> error; 
  Eigen::Matrix<double, 6, 1> velocity_d;
  Eigen::Matrix<double, 6, 1> dvelocity_d;
  Eigen::Matrix<double, 6, 1> velocity;
  Eigen::Matrix<double, 6, 1> error_velocity; 
  double tank_max{20.0};
  double tank_min{0.05};
  double tank{0.0};
 private:
  double angle{0};
  //double omega{0};
  double domega{0};
  ros::Publisher pub;
  // 保护函数
 Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d); 
};

}  // namespace unified_controllers
