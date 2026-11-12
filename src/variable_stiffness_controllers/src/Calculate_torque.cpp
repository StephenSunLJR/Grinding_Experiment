#include "../include/Calculate_torque.h"
#include "../include/franka_utils/pseudo_inversion.h"
#include "../include/franka_utils/Dynamics.h"
#include "../include/franka_model.h"
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <franka/robot_state.h>
#include "std_msgs/String.h"
namespace variable_stiffness_controllers {
bool Calculate_torque::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
   std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
return true;
                                }

void Calculate_torque::starting(const ros::Time& /*time*/) {

    ros::NodeHandle nhpub;
    pub =nhpub.advertise<variable_stiffness_controllers::compliance_robot>("/robotNew",10);

    franka::RobotState robot_state = state_handle_->getRobotState(); //这个是获取机械臂状态的
    t=ros::Duration(0.0);
    q_initial=Eigen::Map<Eigen::Matrix<double ,7,1>>(robot_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    q_desire=q_initial;
    dq_desire.setZero();
    ddq_desire.setZero();
    Kd.setZero();
    Kp.setZero();
    Kp.diagonal()<<600,600,600,600,250,150,50;
    Kd.diagonal()<<50,50,50,20,20,20,10;
}

void Calculate_torque::update(const ros::Time& /*time*/, const ros::Duration& period) {
    t+=period;
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double ,7,1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double,7,1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double,7,1>> tau_J_d(robot_state.tau_J.data());
    Eigen::Map<Eigen::Matrix<double ,7,7>>mass(model_handle_->getMass().data());
    Eigen::Map<Eigen::Matrix<double ,7,1>> coriolis(model_handle_->getCoriolis().data());
    
    double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * t.toSec()))* 0.2;
    double omega = M_PI /16 *0.2 * M_PI / 5 * std::sin(M_PI / 5 * t.toSec());
    double domega=M_PI / 5*M_PI /16 *0.2 * M_PI / 5 * std::cos(M_PI / 5 * t.toSec());
    // q_desire[6]=q_desire[6]+delta_angle;
    // dq_desire[6]=dq_desire[6]+omega;
    // ddq_desire[6]=ddq_desire[6]+domega;

    error=q_desire-q;
    derror=dq_desire-dq;
    
    cout<<"q_desire="<<q_desire.transpose()<<endl;
    cout<<"q="<<q.transpose()<<endl;
    cout<<"error="<<error.transpose()<<endl;
    // tau=mass*(ddq_desire+Kp*error+Kd*derror)+coriolis;
    cout<<"mass="<<mass<<endl;
    // tau=mass*(ddq_desire+Kp*error+Kd*derror)+coriolis;
    tau=Kp*error+Kd*derror;
    tau=saturateTorqueRate(tau,tau_J_d);
    cout<<"tau="<<tau.transpose()<<endl;
    for(size_t i=0;i<7;i++)
    {
        joint_handles_[i].setCommand(tau[i]);
    }

    
}


Eigen::Matrix<double, 7, 1> Calculate_torque::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}
}

PLUGINLIB_EXPORT_CLASS(variable_stiffness_controllers::Calculate_torque,
                       controller_interface::ControllerBase)

