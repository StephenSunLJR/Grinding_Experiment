#include "../include/icvs_energy_mujoco.h"
#include <cmath>
#include <urdf/model.h>

#include <pluginlib/class_list_macros.h>
#include "../include/franka_utils/Dynamics.h"
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp> //加载urdf文件
#include <pinocchio/algorithm/kinematics.hpp> //正向运动学
#include <pinocchio/algorithm/frames.hpp> //坐标系更新与末端位姿
#include <pinocchio/algorithm/jacobian.hpp> //雅可比矩阵
#include <pinocchio/algorithm/crba.hpp> //质量矩阵
#include <pinocchio/algorithm/rnea.hpp> //科式力与非线性项
#include <Eigen/Dense>
#include <ros/ros.h>

namespace variable_stiffness_controllers {

bool IcvsEnergyMujoco::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  std::string arm_id;
  std::vector<std::string> joint_names;
  
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("IcvsEnergyMujoco: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("IcvsEnergyMujoco: Invalid joint_names parameters provided");
    return false;
  }

  // 1. 获取通用的 Effort 接口 (抛弃 FrankaModelInterface)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("IcvsEnergyMujoco: Error getting effort joint interface");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
  }

  // 2. 初始化 KDL 运动学解析器 (从参数服务器读取机械臂模型)
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", node_handle)) {
    ROS_ERROR("IcvsEnergyMujoco: Failed to parse urdf file");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)) {
    ROS_ERROR("IcvsEnergyMujoco: Failed to construct kdl tree");
    return false;
  }
  
  std::string root_name = arm_id + "_link0";
  std::string tip_name = arm_id + "_link8"; // 通常以 link8 为末端，可根据 URDF 调整
  if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_)) {
    ROS_ERROR_STREAM("IcvsEnergyMujoco: Failed to get KDL chain from " << root_name << " to " << tip_name);
    return false;
  }
  
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  // 3. 读取你的算法参数
  node_handle.getParam("sigma", sigma_);
  node_handle.getParam("xt", xt_);
  node_handle.getParam("kc", kc_);
  node_handle.getParam("kt", kt_);
  node_handle.getParam("ktmax", kt_max);
  node_handle.getParam("ktmin", kt_min);
  node_handle.getParam("min_tank_level", min_tank_level_);
  node_handle.getParam("max_tank_level", max_tank_level_);
  node_handle.getParam("lambda", lambda);

  return true;
}

void IcvsEnergyMujoco::starting(const ros::Time& /*time*/) {
  ros::NodeHandle nhpub;
  pub = nhpub.advertise<variable_stiffness_controllers::compliance_robot>("/robotNew", 10);
  sub = nhpub.subscribe<std_msgs::String>("emg_signal", 100, &IcvsEnergyMujoco::doEmg, this, ros::TransportHints().reliable().tcpNoDelay());

  KDL::JntArray q_kdl(7);
  KDL::JntArray dq_kdl(7);
  for (size_t i = 0; i < 7; ++i) {
    q_init_array[i] = joint_handles_[i].getPosition();
    dq_init_array[i] = joint_handles_[i].getVelocity();
    ddq_init_array[i] = 0.0;
    q_kdl(i) = q_init_array[i];
    dq_kdl(i) = dq_array[i];
    last_tau_cmd_[i] = 0.0;
  }

  // 使用 KDL 计算初始正向运动学 (替代原先的 O_T_EE)
  KDL::Frame frame_ee;
  fk_solver_->JntToCart(q_kdl, frame_ee);
  position_d_ << frame_ee.p.x(), frame_ee.p.y(), frame_ee.p.z();
  double x, y, z, w;
  frame_ee.M.GetQuaternion(x, y, z, w);
  orientation_d_ = Eigen::Quaterniond(w, x, y, z);

  qd_array = q_init_array;
  dqd_array = dq_init_array;
  ddqd_array = ddq_init_array;

  t = ros::Duration(0.0);
  md.setZero();
  dd.setZero();
  md.diagonal() << 16.0, 16.0, 16.0, 6.0, 6.0, 6.0;
  dd.diagonal() << 30.0, 30.0, 30.0, 20.0, 20.0, 20.0;
  
  kc_matrix.setZero();
  kt_matrix.setZero();
  kc_matrix.diagonal() << kc_, kc_, kc_, kc_, kc_, kc_;
  kt_matrix.diagonal() << kt_, kt_, kt_, kt_, kt_, kt_, kt_;
  I.setIdentity();
  r.setZero();
  r.diagonal() << 1, 1, 1, 1, 1, 1;

  limit_max = position_d_[2] + 0.15;
  limit_min = position_d_[2] - 0.15;
  a_circle = position_d_[0];
  b_circle = position_d_[1];
  c_circle = position_d_[2];
  
  f_tau.setZero();
  jacobian_old.setZero(); 
  wt_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  tau_cmd.setZero();
  last_kt = kt_;
  firstUpdate = true;
}

void IcvsEnergyMujoco::update(const ros::Time& /*time*/, const ros::Duration& period) {
  count += period.toSec();
  t += period; 
  
  KDL::JntArray q_kdl(7);
  KDL::JntArray dq_kdl(7);
  for (size_t i = 0; i < 7; ++i) {
    q_array[i] = joint_handles_[i].getPosition();
    dq_array[i] = joint_handles_[i].getVelocity();
    q_kdl(i) = q_array[i];
    dq_kdl(i) = dq_array[i];
  }

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());

  // 1. 使用 KDL 实时求解位姿 (替代 O_T_EE)
  KDL::Frame frame_ee;
  fk_solver_->JntToCart(q_kdl, frame_ee);
  Eigen::Vector3d position(frame_ee.p.x(), frame_ee.p.y(), frame_ee.p.z());
  double qx, qy, qz, qw;
  frame_ee.M.GetQuaternion(qx, qy, qz, qw);
  Eigen::Quaterniond orientation(qw, qx, qy, qz);

  // 2. 使用 KDL 实时求解雅可比矩阵
  KDL::Jacobian jac_kdl(7);
  jac_solver_->JntToJac(q_kdl, jac_kdl);
  Eigen::Matrix<double, 6, 7> jacobian = jac_kdl.data; 

  Eigen::Matrix<double, 6, 1> error;

  // 轨迹生成
  double x_tar = a_circle;
  double y_tar = b_circle;
  double z_tar = c_circle;
  position_d_ << x_tar, y_tar, z_tar;
  error.head(3) << position - position_d_;
  
  velocity_d_ << 0, 0, 0, 0, 0, 0; 
  dvelocity_d_ << 0, 0, 0, 0, 0, 0; 
  error_velocity = jacobian * dq - velocity_d_;

  // 姿态误差计算
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  
  // 转换到世界坐标系并转回矩阵
  Eigen::Matrix3d R_ee = orientation.toRotationMatrix();
  error.tail(3) << -R_ee * error.tail(3);

  Eigen::Matrix<double,6,7> jacobian_dot_new;
  double r1 = 0.01;
  if(firstUpdate) {
    S1 = jacobian;
    jacobian_dot_new.setZero();
    S1_dot.setZero();
  } else {
    jacobian_dot_new = (jacobian - jacobian_old) / period.toSec();
    S1_dot = (jacobian - S1) / r1;
    S1 = S1_dot * period.toSec() + S1;
  }
  jacobian_old = jacobian;
  firstUpdate = false;
  
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_interactive_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  Eigen::MatrixXd jacobian_pinv;
  franka_interactive_controllers::pseudoInverse(jacobian, jacobian_pinv);

  // 3. 动力学补偿 (完美复用你写的 Dynamics.h 里的函数)
  KDL::Vector grav(0.0,0.0,0.0);//这里没添加重力学补偿
  KDL::ChainDynParam dyn_param(kdl_chain_,grav);

  KDL::JntSpaceInertiaMatrix M_kdl(7);
  KDL::JntArray C_kdl(7);
  //计算质量矩阵与科式力
  dyn_param.JntToMass(q_kdl,M_kdl);
  dyn_param.JntToCoriolis(q_kdl,dq_kdl,C_kdl);

  Eigen::MatrixXd m_mario = M_kdl.data;
  Eigen::Matrix<double,6,6> miu;
  miu.setZero();

  Eigen::MatrixXd m_mario_pinv;
  franka_interactive_controllers::pseudoInverse(m_mario,m_mario_pinv);

  Eigen::MatrixXd Lambda;
  franka_interactive_controllers::pseudoInverse((jacobian * m_mario_pinv * jacobian.transpose()), Lambda);

  wt_ = -(kt_matrix * error) / (xt_);
  
  double T = 0.5 * xt_ * xt_;
  kt_ = 100;
  kt_matrix.diagonal() << kt_, kt_, kt_, 0, 0, 0; 

  if(T > min_tank_level_) {
    dela_xt = 0; 
  } else {
    dela_xt = min_tank_level_ - sqrt(2 * T * min_tank_level_ - T * T);
  } 

  wt_ = -(kt_ * xt_ * error) / (xt_ * xt_ + dela_xt);
  Eigen::MatrixXd dxt1_ = ((dd - a * Lambda - r) * error_velocity + (kc_matrix - a * miu - a * r) * error - wt_ * xt_) * (error_velocity + a * error).transpose();
  
  dxt1_(0,0) = dxt1_(0,0) - lambda * xt_;
  double dxt_ = dxt1_(0,0);
  xt_ = xt_ + dxt_ * period.toSec();

  f_tau = Lambda * dvelocity_d_ + miu * velocity_d_ - dd * error_velocity - kc_matrix * error;
  
  tau_cmd = jacobian.transpose() * f_tau; 

  // 4. 力矩限幅 (使用内部缓存的 last_tau_cmd_)
  tau_cmd = saturateTorqueRate(tau_cmd, last_tau_cmd_);
  last_tau_cmd_ = tau_cmd; // 更新缓存

  // 发送底层力矩
  for(size_t i = 0; i < 7; i++) {
     joint_handles_[i].setCommand(tau_cmd[i]); 
     robot.taud[i] = tau_cmd[i];
  }
  ROS_ERROR_STREAM_THROTTLE(0.5, "tau_cmd: " << tau_cmd.transpose());

  // 数据打包与发布
  Eigen::Matrix<double,1,1> change;
  change.setOnes();
  Eigen::MatrixXd v = 0.5 * error_velocity.transpose() * Lambda * error_velocity + 0.5 * error.transpose() * (kc_matrix + kt_matrix) * error + 0.5 * xt_ * xt_ * change;
  
  robot.v = v(0,0);
  robot.position[0] = position[0];
  robot.position[1] = position[1];
  robot.position[2] = position[2];
  robot.position[3] = orientation.x();
  robot.position[4] = orientation.y();
  robot.position[5] = orientation.z();
  
  robot.positiond[0] = position_d_[0];
  robot.positiond[1] = position_d_[1];
  robot.positiond[2] = position_d_[2];
  robot.positiond[3] = orientation_d_.x();
  robot.positiond[4] = orientation_d_.y();
  robot.positiond[5] = orientation_d_.z();
  
  robot.kd = kc_ + (kt_ * xt_ * xt_) / (xt_ * xt_ + dela_xt);
  robot.k_desire = kc_ + kt_;
  robot.s[0] = T;
  robot.s[1] = xt_;
  robot.T_max = max_tank_level_;
  robot.T_min = min_tank_level_;
  robot.sumerror = error.transpose() * error;
  robot.t = t.toSec();
  for(size_t i=0; i<6; i++) {
    robot.positionerror[i] = error[i];
  }

  if(count > 0.1) {
    count = 0;
    pub.publish(robot);
  }
}

void IcvsEnergyMujoco::doEmg(const std_msgs::String::ConstPtr& msg_p) {
  int flag = int(*(msg_p->data.c_str()));
  switch (flag) {
  case '0': kt_ = kt_; break;
  case '1': kt_ = kt_; break;
  case '2': {
    if(kt_ < kt_max) {
       kt_ = last_kt + 50.5;  
       last_kt = kt_;
       kt_matrix.diagonal() << kt_, kt_, kt_, 0, 0, 0;
    }
    break;
  }
  case '3': {
    if(kt_ > kt_min) {
      kt_ = last_kt - 50.5; 
      last_kt = kt_;
      kt_matrix.diagonal() << kt_, kt_, kt_, 0, 0, 0;
    }
    break;
  }
  default: break;
  }
}

Eigen::Matrix<double, 7, 1> IcvsEnergyMujoco::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& last_tau_cmd) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - last_tau_cmd[i];
    tau_d_saturated[i] = last_tau_cmd[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax); 
    
    if(i < 4) {
        tau_d_saturated[i] = std::max(std::min(tau_d_saturated[i], 30.0), -30.0);
    } else {
        tau_d_saturated[i] = std::max(std::min(tau_d_saturated[i], 8.0), -8.0);
    }
  }
  return tau_d_saturated;
}

}  // namespace variable_stiffness_controllers

PLUGINLIB_EXPORT_CLASS(variable_stiffness_controllers::IcvsEnergyMujoco, controller_interface::ControllerBase)