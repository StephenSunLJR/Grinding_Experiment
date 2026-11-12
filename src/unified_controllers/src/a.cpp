#include "../include/unified.h"
#include <cmath>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <franka/robot_state.h>
#include "../include/pseudo_inversion.h"
#include "../include/franka_model.h"

namespace unified_controllers {
bool UnifiedController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::string arm_id;
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("arm_id", arm_id)) {
       ROS_ERROR("UnifiedController: Could not read parameter arm_id");
       return false;
     }
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "UnifiedController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "UnifiedController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "UnifiedController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("UnifiedController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "UnifiedController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "UnifiedController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "UnifiedController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void UnifiedController::starting(const ros::Time& /*time*/) {
  ros::NodeHandle nhpub;
  pub =nhpub.advertise<unified_controllers::unified_robot>("/robotNewm",10);
  franka::RobotState robot_state = state_handle_->getRobotState(); //这个是获取机械臂状态的
  q_init_array= robot_state.q;
  dq_init_array=robot_state.dq;
  ddq_init_array={0,0,0,0,0,0,0}; //开始加速度设为0
  qd_array=q_init_array;
  dqd_array=dq_init_array;
  ddqd_array=ddq_init_array;
  last_dq_array=dq_init_array;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> last_dq(last_dq_array.data()); 
  t=ros::Duration(0.0);
  count=0;
  //第一级任务赋初值
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(robot_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  a_circle=position_d_[0];
  b_circle=position_d_[1];
  c_circle=position_d_[2];
  //ROS_INFO("position_d: %f, %f, %f", position_d_[0],position_d_[1],position_d_[2]);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> f_initial(robot_state.O_F_ext_hat_K.data());
  
  fe_last=f_initial;
  f_init=f_initial;
  f_end_last.setZero();
  dfd.setZero();
  dfe.setZero();
  fd.setZero();
  f_end.setZero(); 
  f_end_target=10.0;
  // f_end[2]=10.0;
  joint_err.setZero();
  jacobian_old_array=model_handle_->getZeroJacobian(franka::Frame::kEndEffector);


  //第一层任务
  kp.setZero();
  kd.setZero();
  ki.setZero();
  kp_target.setZero();
  ki_target.setZero();
  kd_target.setZero();
  Damping.setZero();
  Stiffiness.setZero();
  Stiffiness_end.setZero();
  kp_target.diagonal()<<0.0, 0.0, 1.25, 0.0, 0.0, 0.0;
  ki_target.diagonal()<<0.0, 0.0, 0.6, 0.0, 0.0, 0.0;
  kd_target.diagonal()<<0.0, 0.0, 0.01, 0.0, 0.0, 0.0;
  Stiffiness_end.diagonal()<<1500.0, 1500.0, 1500.0, 200.0, 200.0, 200.0;
  Damping.diagonal()<<20.0,20.0, 20.0, 5.0, 5.0, 5.0;
  fi.setZero();
  fi[2]=5.0;
  xt=100.0;
  tank=0.5*xt*xt;
  tank_max=20.0;
  tank_min=0.05;

  dxt.setZero();
  alpha=1.0;
  gamma=0.5;
 
  
}
std::ofstream outfile1;
int time1;

void UnifiedController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  //ROS_INFO("执行UnifiedController");
  count+=period.toSec();
  t+= period; 
  franka::RobotState robot_state = state_handle_->getRobotState();
  q_array=robot_state.q;
  dq_array=robot_state.dq; 
  mass_array = model_handle_->getMass();
  coriolis_array = model_handle_->getCoriolis();
  gravity_array = model_handle_->getGravity();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double,7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data()); 
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());  
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double,7, 1>> q(q_array.data());
  Eigen::Map<Eigen::Matrix<double,7, 1>> dq(dq_array.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Matrix4d Map1(robot_state.O_T_EE.data());
  Eigen::Matrix<double,3,3>R = Map1.block(0,0,3,3);
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double,6, 1>> f_ext(robot_state.O_F_ext_hat_K.data());
  Eigen::Map<Eigen::Matrix<double,7, 1>> tau_extm(robot_state.tau_ext_hat_filtered.data());
  Eigen::Matrix7d C =CoriolisMatrix(q,dq);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_old_array.data()); 
  //std::cout<<"q: "<<q.transpose()<<endl;
  /*平面*/
  // double x=a_circle;
  // double y=b_circle+0.15*sin(0.2*t.toSec());
  // double z=c_circle;
  // double dx =0.0;
  // double dy =0.2*0.15*cos(0.2*t.toSec());
  // double dz =0.0;
  // double ddx =0.0;
  // double ddy =0.2*0.2*0.15*-sin(0.2*t.toSec());
  // double ddz =0.0;
  /*平面*/
  /*曲面*/
  double radius_y=(abs(b_circle)+abs(b_circle))/2.0;
  //std::cout<<"radius_y: "<<radius_y<<endl;
  double x = a_circle;
  // double y =radius_y*sin(factor_circle*t.toSec()+asin(b_circle/radius_y));
  double A = (0.10+abs(b_circle))/2.0; 
  double y = b_circle+A*(1-cos(factor_circle*t.toSec()));
  //std::cout<<"b_circle1: "<<b_circle<<endl;
  //std::cout<<"asin: "<<asin(b_circle/radius_y)<<endl;
  // double y = b_circle+radius_y*sin(factor_circle*t.toSec());
  //std::cout<<"y :"<<y<<endl;
  //double z =c_circle;
  double z = a1*sin(b1*y+c1) +a2*sin(b2*y+c2) +a3*sin(b3*y+c3);
  double dx =0.0;
  // double dy = radius_y*factor_circle*cos(factor_circle*t.toSec());
  //double dy = radius*factor_circle*cos(factor_circle*t.toSec()+asin(b_circle/radius_y));
  double dy = A*factor_circle*sin(factor_circle*t.toSec());
  double dy_r = A*factor_circle*sin(factor_circle*t.toSec());
  if(dy_r<0.00001)
  {
    if(abs(dy_r)<0.00001){
      dy_r=1;
    }
    else
    {
      dy_r=abs(dy_r);
    }
    dy_r=1;
  }
  //double dz=0.0;
  double dz = (a1*b1*cos(b1*y+c1)+a2*b2*cos(b2*y+c2)+a3*b3*cos(b3*y+c3))*dy;
  double dz_r = (a1*b1*cos(b1*y+c1)+a2*b2*cos(b2*y+c2)+a3*b3*cos(b3*y+c3))*dy_r;
  double ddx =0.0;
  // double ddy = -radius_y*factor_circle*factor_circle*sin(factor_circle*t.toSec());
  //double ddy = -radius*factor_circle*factor_circle*sin(factor_circle*t.toSec()+asin(b_circle/radius_y));
  double ddy = A*factor_circle*factor_circle*cos(factor_circle*t.toSec());;
  //double ddz=0.0;
  double ddz = (-a1*b1*b1*sin(b1*y+c1)-a2*b2*b2*sin(b2*y+c2)-a3*b3*sin(b3*y+c3))*dy + (a1*b1*cos(b1*y+c1)+a2*b2*cos(b2*y+c2)+a3*b3*cos(b3*y+c3))*ddy;
  // double xita=0.0;
  // if(y <0)
  // {
  //   xita= -(M_PI-atan2((a1*b1*cos(b1*y+c1)+a2*b2*cos(b2*y+c2)+a3*b3*cos(b3*y+c3)),1));
  // }
  // if(y ==0)
  // {
  //   xita= -M_PI;
  // }
  // if(y >0)
  // {
  //   xita= M_PI+atan2((a1*b1*cos(b1*y+c1)+a2*b2*cos(b2*y+c2)+a3*b3*cos(b3*y+c3)),1);
  // }
  // tf2::Quaternion qtn;
  // qtn.setRPY(xita,0,0);
  // orientation_d_.x()=qtn.getX();
  // orientation_d_.y()=qtn.getY();
  // orientation_d_.z()=qtn.getZ();
  // orientation_d_.w()=qtn.getW();
 
  /*旋转向量[rx ry rz]*/
  Eigen::Vector3d rx;
  Eigen::Vector3d ry;
  Eigen::Vector3d rz;
  //rx<< R.block(0,0,3,1);
  rx <<1,0,0;
  ry<< dx, dy_r, dz_r;
  ry<<-ry;
  rx.normalize();
  ry.normalize();
  rz<<rx.cross(ry);
  rz.normalize();
  Eigen::Matrix<double,3,3> Rc;
  Rc.setZero();
  for(int i=0;i<3;i++)
  {
    Rc(i,0)=rx(i);
    Rc(i,1)=ry(i);
    Rc(i,2)=rz(i);
  }
  // if(flag)
  // {
  //   Rc=R;
  //   flag=false;
  //   b_circle=position_d_[1];
  // }
  
  Eigen::Quaterniond quaternion(Rc);
  orientation_d_=quaternion;
  

  //orientation_d_=quaternion;

  

  // std::cout<<"Rc: "<<endl<<Rc<<endl;
  // std::cout<<"R: "<<endl<<R<<endl;
  // std::cout<<endl<<endl;

  /*旋转向量*/
  

  // std::cout<<"orientation_d_: "<<orientation_d_.x()<<" "<<orientation_d_.y()<<" "<<orientation_d_.z()<<" "<<orientation_d_.w()<<endl;
  // std::cout<<"orientation: "<<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<orientation.w()<<endl;
  
  /*曲面*/
  //std::cout<<"y :"<<y<<endl;
  position_d_<< x,y,z;  
  //std::cout<<"positiond1: "<<position_d_<<endl;
  velocity_d <<dx, dy,dz,0.0,0.0,0.0;
  dvelocity_d <<ddx, ddy,ddz,0.0,0.0,0.0;
  // position_d_<<a_circle,b_circle,c_circle;
  // velocity_d.setZero();
  // dvelocity_d.setZero();
  velocity<<jacobian*dq;
  error.head(3) << position - position_d_;
  error_velocity <<velocity - velocity_d ;
  //std::cout<<"b_circle2: "<<b_circle<<endl;
  // std::cout<<"position: "<<position<<endl;
  // std::cout<<"positiond2: "<<position_d_<<endl;

  // orientation error
  if (orientation.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);

  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  std::cout<<"error.tail(3): "<<error.tail(3)<<endl;
 



  // if(tank>=tank_min)
  // {
  //   alpha=1.0;
  // }
  // else
  // {
  //   alpha=0.0;
  // }
  // if(tank<= tank_max)
  // {
  //   beta = 1.0;
  // }
  // else
  // {
  //   beta = 0.0;
  // }
  double filter_params_=0.001;
  kp.diagonal() = filter_params_ * kp_target.diagonal() + (1.0 - filter_params_) * kp.diagonal();
  ki.diagonal() = filter_params_ * ki_target.diagonal() + (1.0 - filter_params_) * ki.diagonal();
  kd.diagonal() = filter_params_ * kd_target.diagonal() + (1.0 - filter_params_) * kd.diagonal();
  f_end[2]=filter_params_ * f_end_target + (1.0 - filter_params_)*f_end[2];
  f_ext=filter_params_ * fe_last + (1.0 - filter_params_) * f_ext;
  //std::cout<<endl<<"kp: "<<kp<<endl;
  /*更新f_end和Stiffiness*/
  Stiffiness_end(2,2)=rho_imp*Stiffiness_end(2,2);
  //std::cout<<"Stiffiness_end: "<<endl<<Stiffiness_end<<endl;
  Eigen::Matrix<double,6,6> R_imp;
  R_imp.setZero();
  R_imp <<R.transpose(),Eigen::MatrixXd::Zero(3,3),
          Eigen::MatrixXd::Zero(3,3),R.transpose();
  Stiffiness<<R_imp*Stiffiness_end*R_imp.transpose();
  //Stiffiness<<Stiffiness_end;
  // std::cout<<"R: "<<endl<<R<<endl;
  // std::cout<<"R_imp: "<<endl<<R_imp<<endl;
  //std::cout<<"Stiffiness: "<<endl<<Stiffiness<<endl;
  // std::cout<<"fd: "<<endl<<R_imp*f_end<<endl;
  Eigen::Matrix<double,6,6> R_frc;
  R_frc.setZero();
  R_frc <<R,Eigen::MatrixXd::Zero(3,3),
          Eigen::MatrixXd::Zero(3,3),R;
  dfd=(f_end-f_end_last)/0.001;
  dfe=(R_frc*f_ext-R_frc*fe_last)/0.001;
  f_end_last=f_end;
  fe_last=f_ext;
  fi=fi+ki*(f_end-R_frc*f_ext)*0.001;
  Eigen::Matrix<double, 6, 1> tau_force;
  Eigen::Matrix<double, 6, 1> tau_force_e;
  tau_force.setZero();
  tau_force<< f_end+kp*(f_end-R_frc*f_ext)+ ki*fi+ kd*(dfd-dfe);  //在末端坐标系下
  //std::cout<<"f_end:  "<<endl<<f_end<<endl;
  //std::cout<<"R_frc*f_ext:  "<<endl<<R_frc*f_ext<<endl;

 // std::cout<<"f_end-R_frc*f_ext: "<<endl<<f_end-R_frc*f_ext<<endl;
 // std::cout<<"kp*(f_end-R_frc*f_ext): "<<endl<<kp*(f_end-R_frc*f_ext)<<endl;
  tau_force<<rho_frc*tau_force;
  tau_force_e<<tau_force;
  tau_force=R_frc.transpose()*tau_force; //转换到基坐标系
  omega=alpha/xt*(tau_force);


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
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv,true,0.2);
  Eigen::MatrixXd jacobian_pinv;
  pseudoInverse(jacobian, jacobian_pinv,true,0.2);
  Eigen::MatrixXd m_mario; 
  Eigen::MatrixXd c_mario; 
  Eigen:: MatrixXd g_mario; 
	m_mario=MassMatrix(q);
  c_mario=CoriolisMatrix(q,dq);
  g_mario=GravityVector(q);  
  Eigen::Matrix<double,6,6>miu=jacobian_transpose_pinv*(c_mario*-m_mario*jacobian_pinv*S1_dot)*jacobian_pinv;
  Eigen::MatrixXd m_mario_pinv;
  pseudoInverse(mass, m_mario_pinv,true,0.2);
  Eigen::MatrixXd Lambda;
  pseudoInverse((jacobian*m_mario_pinv*jacobian.transpose()), Lambda,true,0.2);


  Eigen::Matrix<double, 6, 1> tau_impedance =Lambda*dvelocity_d+miu*velocity_d-Damping*error_velocity-Stiffiness*error;
  tau_cmd=jacobian.transpose()*(tau_impedance+tau_force);
  //tau_cmd=jacobian.transpose()*(tau_impedance);


  Eigen::MatrixXd ut = -omega.transpose()*error_velocity;
  dxt=beta/xt*(error_velocity.transpose()*Damping*error_velocity)+ ut;  
  xt=xt+dxt[0]*0.001;
  tank= 0.5 *xt*xt;
 //std::cout<<"tau_cmd1111: "<<endl<<tau_cmd<<endl;
 Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
 tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);//力矩保护
 //std::cout<<"tau_cmd222: "<<endl<<tau_cmd<<endl;
 for(size_t i=0;i<7;i++)
  {
     //ROS_INFO("tau_cmd: %f", tau_cmd[i]);
    joint_handles_[i].setCommand(tau_cmd[i]); //发送力矩
    robot.tau[i]=tau_cmd[i];
  }
  for(size_t i=0;i<6;i++)
  {
    robot.f_b_ext[i]=(R_frc*f_ext)[i];
    robot.f_d_ext[i]=f_end[i];
    robot.tau_force[i]=tau_force[i];
    robot.tau_force_e[i]=tau_force_e[i];
  }
  robot.xd=position_d_[0];
  robot.x=position[0];
  robot.xerr=position[0]-position_d_[0];
  robot.yd=position_d_[1];
  robot.y=position[1];
  robot.yerr=position[1]-position_d_[1];
  robot.zd=position_d_[2];
  robot.z=position[2];
  robot.zerr=position[2]-position_d_[2];
  robot.alphaerr=error[3];
  robot.betaerr=error[4];
  robot.gammaerr=error[5];
  robot.fd=f_end[2];
  robot.fext=(R_frc*f_ext)[2];
  robot.ferr=(R_frc*f_ext)[2]-f_end[2];
  robot.tank=tank;
  robot.dxt=dxt[0];
 if(count>0.1)
  {
    count=0.0;
    pub.publish(robot);} 
}

Eigen::Matrix<double, 7, 1> UnifiedController::saturateTorqueRate(
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




}  // namespace unified_controllers
//注册插件
PLUGINLIB_EXPORT_CLASS(unified_controllers::UnifiedController,
                       controller_interface::ControllerBase)
