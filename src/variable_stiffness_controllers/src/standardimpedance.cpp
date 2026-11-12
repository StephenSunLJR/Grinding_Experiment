#include "../include/standardimpedance.h"
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
bool standardimpedance::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::string arm_id;
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("arm_id", arm_id)) {
       ROS_ERROR("standardimpedance: Could not read parameter arm_id");
       return false;
     }
  
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "standardimpedance: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "standardimpedance: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "standardimpedance: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("standardimpedance: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "standardimpedance: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "standardimpedance: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "standardimpedance: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  
  
  // if (!node_handle.getParam("md_gains", md_gains_) || md_gains_.size() != 6) {
  //   ROS_ERROR(
  //       "standardimpedance:  Invalid or no md_gain parameters provided, aborting "
  //       "controller init!");
  //   return false;
  // }
  
  // if (!node_handle.getParam("dd_gains", dd_gains_) || dd_gains_.size() != 6) {
  //   ROS_ERROR(
  //       "standardimpedance:  Invalid or no dd_gain parameters provided, aborting "
  //       "controller init!");
  //   return false;
  // }

  // if (!node_handle.getParam("kd_gains", kd_gains_) || kd_gains_.size() != 6) {
  //   ROS_ERROR(
  //       "standardimpedance:  Invalid or no kd_gain parameters provided, aborting "
  //       "controller init!");
  //   return false;
  // }

  if (!node_handle.getParam("sigma", sigma_) ) {
    ROS_ERROR(
        "standardimpedance:  Invalid or no sigma_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("xt", xt_) ) {
    ROS_ERROR(
        "standardimpedance:  Invalid or no xt_ parameters provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("kc", kc_) ) {
    ROS_ERROR(
        "standardimpedance:  Invalid or no kc_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kt", kt_) ) {
    ROS_ERROR(
        "standardimpedance:  Invalid or no kt_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("ktmax", kt_max) ) {
    ROS_ERROR(
        "standardimpedance:  Invalid or no kt_max parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("ktmin", kt_min) ) {
    ROS_ERROR(
        "standardimpedance:  Invalid or no kt_max parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("min_tank_level", min_tank_level_) ) {
    ROS_ERROR(
        "standardimpedance:  Invalid or no min_tank_level parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("max_tank_level", max_tank_level_) ) {
    ROS_ERROR(
        "standardimpedance:  Invalid or no max_tank_level parameters provided, aborting "
        "controller init!");
    return false;
  }
  
  return true;
}

void standardimpedance::starting(const ros::Time& /*time*/) {
  ros::NodeHandle nhpub;
  pub =nhpub.advertise<variable_stiffness_controllers::compliance_robot>("/robotNew",10);
  sub = nhpub.subscribe<std_msgs::String>("emg_signal",100,&standardimpedance::doEmg,
                                                          this, ros::TransportHints().reliable().tcpNoDelay());


  franka::RobotState robot_state = state_handle_->getRobotState(); //这个是获取机械臂状态的
  q_init_array= {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};//robot_state.q;
  dq_init_array=robot_state.dq;
  ddq_init_array={0,0,0,0,0,0,0}; //开始加速度设为0

  qd_array=q_init_array;
  dqd_array=dq_init_array;
  ddqd_array=ddq_init_array;
  
  t=ros::Duration(0.0);
  // md.diagonal()<<md_gains_[0],md_gains_[1],md_gains_[2],md_gains_[3],md_gains_[4],md_gains_[5];
  // dd.diagonal()<<dd_gains_[0],dd_gains_[1],dd_gains_[2],dd_gains_[3],dd_gains_[4],dd_gains_[5];
  // kd.diagonal()<<kd_gains_[0],kd_gains_[1],kd_gains_[2],kd_gains_[3],kd_gains_[4],kd_gains_[5];
  md.diagonal()<<16.0, 16.0, 16.0, 6.0, 6.0, 6.0;
  dd.diagonal()<<15.0, 15.0, 15.0, 15.0, 15.0, 15.0;
  kd.diagonal()<<50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
  
  I.setIdentity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(robot_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  
  position_d_ = initial_transform.translation();
  limit_max=position_d_[2]+0.15;
  limit_min=position_d_[2]-0.15;
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  a_circle=position_d_[0];
  b_circle=position_d_[1];
  c_circle=position_d_[2];
  f_tau.setZero();
  jacobian_old_array=model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  wt_ << 0.1,0.1,0.1,0.1,0.1,0.1;
  tau_cmd.setZero();
  last_kt=kt_;
  Vfunction.setZero();

}

void standardimpedance::update(const ros::Time& /*time*/, const ros::Duration& period) {
  count+=period.toSec();
  t+= period; 
  franka::RobotState robot_state = state_handle_->getRobotState();
  //dqd_array=robot_state.dq_d;
  //ddqd_array=robot_state.ddq_d;
  q_array=robot_state.q;
  dq_array=robot_state.dq;
  //tau_ext_array=robot_state.tau_ext_hat_filtered; //robot_state.tau_ext_hat_filtered 
  tau_J_d_array=robot_state.tau_J_d;
  
  mass_array = model_handle_->getMass();
  coriolis_array = model_handle_->getCoriolis();
  gravity_array = model_handle_->getGravity();
  //Eigen映射
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
  //Eigen::Map<Eigen::Matrix<double, 7, 1>> qd(qd_array.data());
  //Eigen::Map<Eigen::Matrix<double, 7, 1>> dqd(dqd_array.data());
  //Eigen::Map<Eigen::Matrix<double, 7, 1>> ddqd(ddqd_array.data());

  //Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext(tau_ext_array.data());
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(tau_J_d_array.data());
  Eigen::Map<Eigen::Matrix<double,7, 7>> mass(mass_array.data());  //质量矩阵
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data()); //直接返回的就是科氏力和向心力
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());  //直接返回重力矩阵

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  //std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kFlange);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());//雅可比矩阵
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_old_array.data()); 
  
 
  
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  


  double x=a_circle;
  double y=b_circle+radius_*sin(factor_circle*t.toSec());
  double z=c_circle-radius_*sin(factor_circle*t.toSec());
  position_d_<<x,y,z;
  error.head(3) << position - position_d_;
  double dx=0.0;
  //double dx=-factor_circle*radius_*cos(factor_circle*t.toSec());
  double dy=factor_circle*radius_*cos(factor_circle*t.toSec());
  double dz=-factor_circle*radius_*cos(factor_circle*t.toSec());

  double ddx=0.0;
  //double ddx=factor_circle*factor_circle*radius_*sin(factor_circle*t.toSec());
  //double ddx=factor_circle*factor_circle*radius_*sin(factor_circle*t.toSec());
  double ddy=-factor_circle*factor_circle*radius_*sin(factor_circle*t.toSec());
  double ddz=factor_circle*factor_circle*radius_*sin(factor_circle*t.toSec());
  velocity_d_<<dx,dy,dz,0,0,0; //角速度设置为0
  dvelocity_d_<<ddx,ddy,ddz,0,0,0; //角加速度设置为0
  error_velocity=jacobian * dq-velocity_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  Eigen::Matrix<double,6,7> jacobian_dot;
  Eigen:: Matrix<double,6,7> jacobian_dot_new;
  double r1 =0.01;
  if(firstUpdate)
  {
    S1=jacobian;
    jacobian_dot_new.setZero();
    S1_dot.setZero();
  }
  else
  {
    jacobian_dot_new =(jacobian-jacobian_dot)/period.toSec();
    S1_dot=(jacobian -S1)/r1;
    S1=S1_dot *period.toSec()+S1;
  }
  jacobian_old_array=jacobian_array;
  firstUpdate=false;
  
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_interactive_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  Eigen::MatrixXd jacobian_pinv;
  franka_interactive_controllers::pseudoInverse(jacobian, jacobian_pinv);

	MatrixXd m_mario; MatrixXd c_mario; MatrixXd g_mario; 
	m_mario=MassMatrix(q);
  c_mario=CoriolisMatrix(q,dq);
  g_mario=GravityVector(q);  
  Eigen::Matrix<double,6,6>miu=jacobian_transpose_pinv*(c_mario*-m_mario*jacobian_pinv*S1_dot)*jacobian_pinv;
  //Eigen::Matrix<double,6,6>miu=jacobian_pinv.transpose()*(c_mario*-m_mario*jacobian_pinv*S1_dot)*jacobian_pinv;

  Eigen::MatrixXd m_mario_pinv;
  franka_interactive_controllers::pseudoInverse(mass, m_mario_pinv);

  Eigen::MatrixXd Lambda;
  franka_interactive_controllers::pseudoInverse((jacobian*m_mario_pinv*jacobian.transpose()), Lambda);
  
  wt_=-(kt_*error)/xt_;
  
  double T=1.0/2.0*xt_*xt_;
  
  //能量罐部分
  
  //double dxt_=(error_velocity.transpose() *dd* error_velocity)*(sigma_/xt_)-(wt_.transpose()*error_velocity);
  Eigen::MatrixXd dxt1_=(error_velocity.transpose() *dd* error_velocity)*(sigma_/xt_);
  dxt1_=dxt1_ - (wt_.transpose()*error_velocity);
  double b_factor =0.005;
  //障碍函数
  dxt1_(0,0)= dxt1_(0,0) -b_factor*xt_/(max_tank_level_-0.5*xt_*xt_);
  double dxt_=dxt1_(0,0);
  xt_=xt_+dxt_*period.toSec();
  
  Eigen::Matrix<double,6,6> kdt;
  kdt.setOnes();
  kdt=(500+500*sin(t.toSec()))*kdt;

  Eigen::Matrix<double,6,6> dkdt;
  dkdt.setOnes();
  dkdt=(-100*cos(t.toSec()))*dkdt;

  f_tau= Lambda*dvelocity_d_+miu*velocity_d_- dd*error_velocity-kdt*error;

  tau_cmd = jacobian.transpose() *f_tau;

  
  tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);//力矩保护
  //std::cout<<"tau_cmd: "<<tau_cmd<<endl;
  for(size_t i=0;i<7;i++)
  {
     joint_handles_[i].setCommand(tau_cmd[i]); //发送力矩
     robot.taud[i]=tau_cmd[i];
  }

  robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform1(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position1(transform1.translation());
  Eigen::Quaterniond orientation1(transform1.linear());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> f_ext(robot_state.O_F_ext_hat_K.data());
  
  // Eigen::MatrixXd dotv=error_velocity.transpose()*f_ext+0.5*error.transpose()*dkdt*error-error_velocity.transpose()*dd*error_velocity;
  // Vfunction=Vfunction+dotv*period.toSec();
  Vfunction=0.5*error_velocity.transpose()*Lambda*error_velocity+0.5*error.transpose()*kdt*error;
  robot.v=Vfunction(0,0);
  robot.position[0]=position1[0];
  robot.position[1]=position1[1];
  robot.position[2]=position1[2];
  robot.position[3]=orientation1.x();
  robot.position[4]=orientation1.y();
  robot.position[5]=orientation1.z();
  
  robot.positiond[0]=position_d_[0];
  robot.positiond[1]=position_d_[1];
  robot.positiond[2]=position_d_[2];
  robot.positiond[3]=orientation_d_.x();
  robot.positiond[4]=orientation_d_.y();
  robot.positiond[5]=orientation_d_.z();
  robot.kd=kdt(0,0);
  robot.s[0]=T;
  robot.s[1]=xt_;
  robot.sumerror= error.transpose()*error;
  robot.t=t.toSec();
  for(size_t i=0;i<6;i++)
  {
    robot.positionerror[i]=error[i];
  }
  if(count>0.1)
  {
    count=0;
  pub.publish(robot);}
  
}
void standardimpedance::doEmg(const std_msgs::String::ConstPtr& msg_p){
    //ROS_INFO("Receive:%s",msg_p->data.c_str());
    // ROS_INFO("我听见:%s",(*msg_p).data.c_str());
  int flag=int(*(msg_p->data.c_str()));
  switch (flag)
  {
  case '0':{
    kt_=kt_;  //松开保持不变
    //std::cout<<"No Change"<<std::endl;
    break;
    }
  case '1':{
    kt_=kt_; //握紧保持不变
    //std::cout<<"No Change"<<std::endl;
    break;
    }
  case '2':{
    if(kt_<kt_max)
    {
       kt_=last_kt+0.5;  
       last_kt=kt_;
       std::cout<<"Sum: "<<kt_<<std::endl;
    }
    break;
    }
  case '3':{
    if(kt_>kt_min) 
    {
      kt_=last_kt-0.5; //减
      last_kt=kt_;
      std::cout<<"Decrease: "<<kt_<<std::endl;
    }
    break;
    }
  default:
    std::cout<<"default no change"<<std::endl;
    //kt_=kt_;  //不变；
    break;
  }

}

Eigen::Matrix<double, 7, 1> standardimpedance::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax); //控制力矩变化率
    //以下的力矩限制可以通过自己的需要更改，详见fci
   if(i==0||i==1||i==2||i==3)
    {
        if(tau_d_saturated[i]>30)
        {
            tau_d_saturated[i]=30;
        }
        if(tau_d_saturated[i]<-30)
        {
            tau_d_saturated[i]=-30;
        }
    }
    else
    {
        if(tau_d_saturated[i]>8)
        {
            tau_d_saturated[i]=8;
        }
        if(tau_d_saturated[i]<-8)
        {
            tau_d_saturated[i]=-8;
        }
    }
  }
  return tau_d_saturated;
}

}  // namespace variable_stiffness_controllers
//注册插件
PLUGINLIB_EXPORT_CLASS(variable_stiffness_controllers::standardimpedance,
                       controller_interface::ControllerBase)
