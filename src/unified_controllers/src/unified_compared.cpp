#include "../include/unified_compared.h"
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
bool UnifiedComparedController ::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::string arm_id;
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("arm_id", arm_id)) {
       ROS_ERROR("UnifiedComparedController: Could not read parameter arm_id");
       return false;
     }
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "UnifiedComparedController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "UnifiedComparedController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "UnifiedComparedController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("UnifiedComparedController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "UnifiedComparedController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "UnifiedComparedController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "UnifiedComparedController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void UnifiedComparedController::starting(const ros::Time& /*time*/) {
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
  f_end_target=8.0;
  // f_end[2]=10.0;
  joint_err.setZero();
  jacobian_old_array=model_handle_->getZeroJacobian(franka::Frame::kEndEffector);


  //第一层任务
  kt.setZero();
  kc.setZero();
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
  ki_target.diagonal()<<0.0, 0.0, 4.0, 0.0, 0.0, 0.0;
  kd_target.diagonal()<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  
  kc.diagonal()<<100.0,100,0,100.0,10.0,10.0,10.0;
  //Stiffiness_end.diagonal()<<1500.0, 2500.0, 2000.0, 200.0, 200.0, 200.0;
  Stiffiness_end.diagonal()<<800.0, 1500.0, 1000.0, 200.0, 200.0, 200.0;
  Damping.diagonal()<<20.0, 20.0, 20.0, 5.0, 5.0, 5.0;
  // Damping.diagonal()<<10.0, 10.0, 10.0, 3.0, 3.0, 3.0;
  K2<<50.0;
  D2<<10.0;
  Kt2 << 100.0;
  fi.setZero();
  fi[2]=5.0;
  tank=0.5*xt*xt;
  // tank_max=20.0;
  // tank_min=0.05;

  dxt.setZero();
  alpha=1.0;
  gamma=0.5;

  B_last.setZero();
  x2_bar <<q_init_array[task2_j1];
  // std::cout<<"x2_bar: "<<x2_bar<<endl;
  
}


void UnifiedComparedController::update(const ros::Time& /*time*/, const ros::Duration& period) {
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
  Eigen::Map<Eigen::Matrix<double,6, 1>> f_ext(robot_state.O_F_ext_hat_K.data());//获取执行器末端的力
  Eigen::Map<Eigen::Matrix<double,7, 1>> tau_extm(robot_state.tau_ext_hat_filtered.data());
  Eigen::Matrix7d C =CoriolisMatrix(q,dq);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_old_array.data()); 
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



  double x=a_circle;
  double y=b_circle+0.1*sin(0.2*t.toSec());
  // double y=b_circle;
  double z=c_circle;

  double dx =0.0;
  double dy =0.2*0.1*cos(0.2*t.toSec());
  // double dy =0.0;
  double dz =0.0;
  double ddx =0.0;
  double ddy =0.2*0.2*0.1*(-sin(0.2*t.toSec()));
  // double ddy =0.0;
  double ddz =0.0;
  /*平面*/


  position_d_<< x,y,z;  
  velocity_d <<dx, dy,dz,0.0,0.0,0.0;
  dvelocity_d <<ddx, ddy,ddz,0.0,0.0,0.0;
  velocity<<jacobian*dq;
  error.head(3) << position - position_d_;
  error_velocity <<velocity - velocity_d ;
  // orientation error
  if (orientation.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
 
  double filter_params_=0.001;
  kp.diagonal() = filter_params_ * kp_target.diagonal() + (1.0 - filter_params_) * kp.diagonal();
  ki.diagonal() = filter_params_ * ki_target.diagonal() + (1.0 - filter_params_) * ki.diagonal();
  kd.diagonal() = filter_params_ * kd_target.diagonal() + (1.0 - filter_params_) * kd.diagonal();
  f_end[2]=filter_params_ * f_end_target + (1.0 - filter_params_)*f_end[2];
  double filter_params_1 =0.1;
  f_ext=filter_params_1 * fe_last + (1.0 - filter_params_1) * f_ext;//低通滤波器

/****************************************************************************************************************************************************/
  Eigen::Matrix<double, 6, 7> J1;
  J1=jacobian;
  Eigen::MatrixXd J1_line = J1;
  Eigen::MatrixXd Lambda1;
  Eigen::MatrixXd M_inv;
  pseudoInverse((mass), M_inv);
  pseudoInverse((J1*M_inv*J1.transpose()), Lambda1);
  Eigen::Matrix<double,6,6> miu;
  
  /****************************************************************************************************************************************************/
  Eigen::Matrix<double,6,6> R_imp;
  R_imp.setZero();
  R_imp <<R.transpose(),Eigen::MatrixXd::Zero(3,3),
          Eigen::MatrixXd::Zero(3,3),R.transpose();
  error_ee << R_imp.transpose()*error; //在末端坐标系下的误差
  //std::cout << "error_ee: "<<error_ee.norm()<<endl;
  error_ee[2]=-error_ee[2];
  // //更新rho_imp和rho_frc, 以末端位置误差
  double d_max=0.025;
  double d_min=0.015;
  if(abs(error_ee[2])> d_max)
  {
    rho_imp=0.0;
    fi[2]=5.0;
  }
  else if(abs(error_ee[2])< d_min)
  {
    rho_imp=1.0;
  }
  else
  {
    rho_imp=0.5*(1-cos( abs((error_ee[2])-d_max)/(d_max-d_min)*M_1_PI ));
    // ROS_INFO("rho_imp: %f", rho_imp);
  }


  rho_frc=rho_imp;
  
  
  Stiffiness<<R_imp*Stiffiness_end*R_imp.transpose();
  kt<<rho_imp*Stiffiness;
  
  

  Eigen::Matrix<double,6,6> R_frc;
  R_frc.setZero();
  R_frc <<R,Eigen::MatrixXd::Zero(3,3),
          Eigen::MatrixXd::Zero(3,3),R;
  dfd=(f_end-f_end_last)/0.001;
  double force_value_change = (R_frc*f_ext)[2];
  double force_value = addValue(force_value_change);
  force_value=force_value_change;
  Eigen::Matrix<double, 6, 1> force;
  force.setZero();
  force << 0.0, 0.0, force_value, 0.0, 0.0, 0.0;
  dfe=(force-fe_last)/0.001;
  f_end_last=f_end;
  fe_last=force;
  fi=fi+ki*(f_end-force)*0.001;
  Eigen::Matrix<double, 6, 1> tau_force;
  Eigen::Matrix<double, 6, 1> tau_force_e;
  tau_force.setZero();
  tau_force<< f_end+kp*(f_end-force)+ ki*fi+ kd*(dfd-dfe);  //在末端坐标系下

  // cout<<"force[2]="<<force[2]<<endl;

  tau_force << rho_frc*tau_force;
  tau_force_e << tau_force;
  tau_force = R_frc.transpose()*tau_force; //转换到基坐标系
  if(tank>=tank_min)
  {
    alpha=1.0;
  }
  else
  {
    alpha=0.0;
  }

  omega = alpha/xt*(-kt*error+tau_force);

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
  Eigen::MatrixXd m_mario_pinv;
  pseudoInverse(mass, m_mario_pinv,true,0.2);
  //求取雅可比的导数
  if(flag)
  {
   d_jacobian=jacobian/0.001;
   flag=false;
   J_line_last=jacobian;
  }
  else
  {
   d_jacobian=(jacobian-J_line_last)/0.001;
   J_line_last=jacobian;
  }

  miu=jacobian_transpose_pinv*(c_mario-mass*jacobian_pinv*d_jacobian)*jacobian_pinv;
  Eigen::Matrix<double, 7, 1> tau1 =J1_line.transpose()*(Lambda1*dvelocity_d+miu*velocity_d-Damping*error_velocity-kc*error+omega*xt);

  tau_cmd=tau1;
  
  double sigma=0.9;
  double b_factor=0.000001;
  Eigen::MatrixXd ut = -omega.transpose()*error_velocity;
  dxt=(sigma*xt)/(xt*xt+tank_min)*(error_velocity.transpose()*Damping*error_velocity)+ ut;  
  //barrier function
  dxt[0]=dxt[0]-b_factor*xt/(tank_max-0.5*xt*xt);
//barrier function
  xt=xt+dxt[0]*0.001;
  Eigen::MatrixXd P=xt*ut;
  double p_flow=P(0,0);
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
    robot.tau1[i]=tau1[i];
    // robot.tau2[i]=tau2[i];
  }
  for(size_t i=0;i<6;i++)
  {
    robot.f_b_ext[i]=(force)[i];
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


  robot.omegad=orientation_d_.w();
  robot.omega=orientation.w();

  robot.alphad=orientation_d_.x();
  robot.alpha=orientation.x();
  robot.alphaerr=error[3];

  robot.betad=orientation_d_.y();
  robot.beta=orientation.y();
  robot.betaerr=error[4];
  robot.gammad=orientation_d_.z();
  robot.gamma=orientation.z();
  robot.gammaerr=error[5];

  robot.fd=f_end[2];
  robot.fext=force[2];
  robot.ferr=force[2]-f_end[2];
  robot.tank=tank;
  robot.dxt=dxt[0];
  robot.rho_frc=rho_frc;
  robot.rho_imp=rho_imp;
  robot.error_ee=error_ee[2];
  robot.P=abs(p_flow);
 if(count>0.1)
  {
    count=0.0;
    pub.publish(robot);} 
}

Eigen::Matrix<double, 7, 1> UnifiedComparedController::saturateTorqueRate(
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

double UnifiedComparedController::addValue(double value){
      buffer.push_back(value);
      sum += value;
      if(buffer.size() > windowSize)
      {
        sum -= buffer.front();
        buffer.erase(buffer.begin());
      }
      return sum/buffer.size();
}


}  // namespace unified_controllers
//注册插件
PLUGINLIB_EXPORT_CLASS(unified_controllers::UnifiedComparedController,
                       controller_interface::ControllerBase)
