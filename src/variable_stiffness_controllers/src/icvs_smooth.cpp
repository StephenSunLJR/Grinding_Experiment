#include "../include/icvs_smooth.h"
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
bool icvs_smooth::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::string arm_id;
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("arm_id", arm_id)) {
       ROS_ERROR("icvs_smooth: Could not read parameter arm_id");
       return false;
     }
  
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "icvs_smooth: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "icvs_smooth: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "icvs_smooth: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("icvs_smooth: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "icvs_smooth: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "icvs_smooth: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "icvs_smooth: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  if (!node_handle.getParam("sigma", sigma_) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no sigma_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("xt", xt_) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no xt_ parameters provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("kc", kc_) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no kc_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kt", kt_) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no kt_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("ktmax", kt_max) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no kt_max parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("ktmin", kt_min) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no kt_max parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("min_tank_level", min_tank_level_) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no min_tank_level parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("max_tank_level", max_tank_level_) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no max_tank_level parameters provided, aborting "
        "controller init!");
    return false;
  }
  
    if (!node_handle.getParam("lambda", lambda) ) {
    ROS_ERROR(
        "icvs_smooth:  Invalid or no max_tank_level parameters provided, aborting "
        "controller init!");
    return false;
  }
  return true;
}

void icvs_smooth::starting(const ros::Time& /*time*/) {
  ros::NodeHandle nhpub;
  pub =nhpub.advertise<variable_stiffness_controllers::compliance_robot>("/robotNew",10);
  sub = nhpub.subscribe<std_msgs::String>("emg_signal",100,&icvs_smooth::doEmg,
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
  md.setZero();
  dd.setZero();
  kd.setZero();
  md.diagonal()<<16.0, 16.0, 16.0, 6.0, 6.0, 6.0;
  dd.diagonal()<<15.0, 15.0, 15.0, 15.0, 15.0, 15.0;
  kd.diagonal()<<50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
  kc_matrix.setZero();
  kt_matrix.setZero();
  kc_matrix.diagonal()<<kc_,kc_,kc_,kc_,kc_,kc_;
  kt_matrix.diagonal()<<kt_,kt_,kt_,kt_,kt_,kt_,kt_;
  I.setIdentity();
  r.setZero();
  r.diagonal()<<1,1,1,1,1,1;
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
  
  // FILE* icvs_smooth_date=fopen("/home/liu/franka_ros_test/catkin_ws/icvs_smooth_date.text","w");
  // fprintf(icvs_smooth_date,"t.toSec()\t,position1[0]\t,position1[1]\t,position1[2]\t,position_d_[0]\t,position_d_[1]\t,position_d_[2]\t,error[1]\t,error[2]\t,error[3]\t,tau_cmd[0]\t,tau_cmd[1]\t,tau_cmd[2]\t,tau_cmd[3]\t,tau_cmd[4]\t,tau_cmd[5]\t,tau_cmd[6]\t,max_tank_level_\t,T,robot.kd\t");
  // fclose(icvs_smooth_date);
}

void icvs_smooth::update(const ros::Time& /*time*/, const ros::Duration& period) {
  count+=period.toSec();
  t+= period; 
  franka::RobotState robot_state = state_handle_->getRobotState();
  q_array=robot_state.q;
  dq_array=robot_state.dq;
  tau_J_d_array=robot_state.tau_J_d;
  
  mass_array = model_handle_->getMass();
  coriolis_array = model_handle_->getCoriolis();
  gravity_array = model_handle_->getGravity();
  //Eigen映射
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
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
  double dy=factor_circle*radius_*cos(factor_circle*t.toSec());
  double dz=-factor_circle*radius_*cos(factor_circle*t.toSec());

  double ddx=0.0;
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
    jacobian_dot_new =(jacobian-jacobian_old)/period.toSec();
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

  //wt_条件
  /*条件1*/
  //  double T=1.0/2.0*xt_*xt_;
  // double dela_xt=0;

  // // kt_=200*sin(M_PI/10*t.toSec());
  //  if( T >min_tank_level_ )
  // {
  //   dela_xt=0;
  // }
  // else
  // {
  //   dela_xt=min_tank_level_-sqrt(2*T*min_tank_level_-T*T);
  // } 
  // wt_=-(kt_*xt_*error)/(xt_*xt_+dela_xt);
  /*条件1*/

  /*条件2*/
  // if(t.toSec()>50.0 && kt_< kt_max)
  // {
  //   kt_=last_kt+5;  
  //   last_kt=kt_;
  //   kt_matrix.diagonal()<<kt_,kt_,kt_,0,0,0;  
  // }
  
//   if(t.toSec()>50.0 && kt_< kt_max)
//   {
//     kt_=2000*(t.toSec()-50);  
//     // last_kt=kt_;
//     kt_matrix.diagonal()<<kt_,kt_,kt_,0,0,0;  
//   }
// else if(kt_> kt_max){
//     kt_=1000;
//     kt_matrix.diagonal()<<kt_,kt_,kt_,0,0,0;  
// }


  wt_=-(kt_matrix*error)/(xt_);
  
  double T=0.5*xt_*xt_;
  kt_=220*sin(M_PI/10*t.toSec());
  kt_matrix.diagonal()<<kt_,kt_,kt_,0,0,0; 
  //wt_条件
  if( T >min_tank_level_ )
  {
    dela_xt=0; 
  }
  else
  {
    dela_xt=min_tank_level_-sqrt(2*T*min_tank_level_-T*T);
  } 
  wt_=-(kt_*xt_*error)/(xt_*xt_+dela_xt);
  /*条件2*/

  
  /* sum energy tank with barrier function */
  // Eigen::MatrixXd dxt1_=(error_velocity.transpose() *dd* error_velocity)*(sigma_* xt_ /(xt_ *xt_+min_tank_level_));
  // dxt1_=dxt1_ - (wt_.transpose()*error_velocity);
  Eigen::MatrixXd dxt1_=((dd-a*Lambda-r)*error_velocity+(kc_matrix-a*miu)*error+wt_*xt_)*(error_velocity+a*error).transpose();
  // double b_factor =0.02;
  //障碍函数
  // dxt1_(0,0)= dxt1_(0,0) -lambda*xt_+b_factor*xt_/(T-min_tank_level_);
  dxt1_(0,0)= dxt1_(0,0) -lambda*xt_;
  /* sum energy tank with barrier function */
  double dxt_=dxt1_(0,0);
  xt_=xt_+dxt_*period.toSec();
  //sum energy tank
  // f_tau= Lambda*dvelocity_d_+miu*velocity_d_- dd*error_velocity- kc_matrix*error+ wt_*xt_;

  // /* no energy tank */
  f_tau= Lambda*dvelocity_d_+miu*velocity_d_- dd*error_velocity- (kc_+kt_)*error;
  // /* no energy tank */
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
  Eigen::Matrix<double,1,1> change;
  change.setOnes();
  //sum energy tank
  Eigen::MatrixXd v=0.5*error_velocity.transpose()*Lambda*error_velocity+0.5*error.transpose()*(kc_matrix+kt_matrix)*error+0.5*xt_*xt_*change;
  //no energy tank
  //Eigen::MatrixXd v=0.5*error_velocity.transpose()*Lambda*error_velocity+0.5*error.transpose()*(kc_+kt_)*error;
  robot.v=v(0,0);
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
  robot.kd=kc_+(kt_*xt_*xt_)/(xt_*xt_+dela_xt);
  robot.k_desire=kc_+kt_;
  robot.s[0]=T;
  robot.s[1]=xt_;
  robot.T_max=max_tank_level_;
  robot.T_min=min_tank_level_;
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
void icvs_smooth::doEmg(const std_msgs::String::ConstPtr& msg_p){
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
       
       kt_=last_kt+50.5;  
       last_kt=kt_;
       kt_matrix.diagonal()<<kt_,kt_,kt_,0,0,0;
       //std::cout<<"Sum: "<<kt_<<std::endl;
    }
    break;
    }
  case '3':{
    if(kt_>kt_min) 
    {
      kt_=last_kt-50.5; //减
      last_kt=kt_;
      kt_matrix.diagonal()<<kt_,kt_,kt_,0,0,0;
      std::cout<<"Decrease: "<<kt_<<std::endl;
    }
    break;
    }
  default:  //kt_=kt_;  //不变；
    std::cout<<"default no change"<<std::endl;
    break;
  }

}

Eigen::Matrix<double, 7, 1> icvs_smooth::saturateTorqueRate(
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
PLUGINLIB_EXPORT_CLASS(variable_stiffness_controllers::icvs_smooth,
                       controller_interface::ControllerBase)
