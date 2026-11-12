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

  // std::cout<<"q: "<<q.transpose()<<endl;
  /*平面*/
  double x=a_circle;
  //double y=b_circle+0.15*sin(0.2*t.toSec());
  double y=b_circle;
  double z=c_circle;

  double dx =0;
  // double dy =0.2*0.15*cos(0.2*t.toSec());
  double dy =0;
  double dz =0.0;
  double ddx =0;
  // double ddy =0.2*0.2*0.15*-sin(0.2*t.toSec());
  double ddy =0;
  double ddz =0.0;
  /*平面*/

  /*曲面*/
  // double radius_y=(abs(b_circle)+abs(b_circle))/2.0;
  // double x = a_circle;
  // double A = (0.10+abs(b_circle))/2.0; 
  // double y = b_circle+A*(1-cos(factor_circle*t.toSec()));
  // double z = a1*sin(b1*y+c1) +a2*sin(b2*y+c2) +a3*sin(b3*y+c3);
  // double dx =0.0;
  // double dy = A*factor_circle*sin(factor_circle*t.toSec());
  // double dy_r = A*factor_circle*sin(factor_circle*t.toSec());
  // if(dy_r<0.00001)
  // {
  //   if(abs(dy_r)<0.00001){
  //     dy_r=1;
  //   }
  //   else
  //   {
  //     dy_r=abs(dy_r);
  //   }
  //   dy_r=1;
  // }
  // double dz = (a1*b1*cos(b1*y+c1)+a2*b2*cos(b2*y+c2)+a3*b3*cos(b3*y+c3))*dy;
  // double dz_r = (a1*b1*cos(b1*y+c1)+a2*b2*cos(b2*y+c2)+a3*b3*cos(b3*y+c3))*dy_r;
  // double ddx =0.0;
  // double ddy = A*factor_circle*factor_circle*cos(factor_circle*t.toSec());;
  // double ddz = (-a1*b1*b1*sin(b1*y+c1)-a2*b2*b2*sin(b2*y+c2)-a3*b3*sin(b3*y+c3))*dy + (a1*b1*cos(b1*y+c1)+a2*b2*cos(b2*y+c2)+a3*b3*cos(b3*y+c3))*ddy;
 
  // /*旋转向量[rx ry rz]*/
  // Eigen::Vector3d rx;
  // Eigen::Vector3d ry;
  // Eigen::Vector3d rz;
  // //rx<< R.block(0,0,3,1);
  // rx <<1,0,0;
  // ry<< dx, dy_r, dz_r;
  // ry<<-ry; 
  // rx.normalize();
  // ry.normalize();
  // rz<<rx.cross(ry);
  // rz.normalize();
  // Eigen::Matrix<double,3,3> Rc;
  // Rc.setZero();
  // for(int i=0;i<3;i++)
  // {
  //   Rc(i,0)=rx(i);
  //   Rc(i,1)=ry(i);
  //   Rc(i,2)=rz(i);
  // }
  // Eigen::Quaterniond quaternion(Rc);
  // orientation_d_=quaternion;
  /*曲面*/

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
  // std::cout<<"error: "<<error.transpose()<<endl;
  /*task 2关节7*/
  // double qd7 = q_init_array[1]+0.1*sin(0.3*t.toSec());
  // double dqd7 = 0.1*0.3*cos(0.3*t.toSec());
  // double ddqd7 = -0.1*0.3*0.3*sin(0.3*t.toSec());
  double qd7 = q_init_array[1];
  double dqd7 = 0;
  double ddqd7 = 0;
  
  double err7_array=q[1]-qd7;
  double derr7_array=dq[1]-dqd7;
  Eigen::Matrix<double,1,1> err7;
  Eigen::Matrix<double,1,1> derr7;
  err7<<err7_array;
  derr7<<derr7_array;
  x2 << q[task2_j1];
  //std::cout<<"q: "<<q<<endl;
  double filter_params_=0.001;
  kp.diagonal() = filter_params_ * kp_target.diagonal() + (1.0 - filter_params_) * kp.diagonal();
  ki.diagonal() = filter_params_ * ki_target.diagonal() + (1.0 - filter_params_) * ki.diagonal();
  kd.diagonal() = filter_params_ * kd_target.diagonal() + (1.0 - filter_params_) * kd.diagonal();
  f_end[2]=filter_params_ * f_end_target + (1.0 - filter_params_)*f_end[2];
  double filter_params_1 =0.1;
  f_ext=filter_params_1 * fe_last + (1.0 - filter_params_1) * f_ext;
  //std::cout<<endl<<"kp: "<<kp<<endl;

  // /* nullspace */
  Eigen::Matrix<double, 6, 7> J1;
  Eigen::Matrix<double, 1, 7> J2;
  J1=jacobian;
  J2.setZero();
  J2[task2_j1]=1.0;
  Eigen::Matrix<double, 6, 7> J1_aug;
  Eigen::Matrix<double, 7, 7> J2_aug;
  J1_aug << J1;
  J2_aug << J1, J2;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd1(J1_aug, Eigen::ComputeFullV | Eigen::ComputeFullV);
  int rank1=svd1.rank();
  Eigen::MatrixXd V1_right=svd1.matrixV();
  Eigen::MatrixXd V1=V1_right.rightCols(J1_aug.cols()-rank1);

  Eigen::MatrixXd VMV_inv1;
  pseudoInverse((V1.transpose()*mass*V1),VMV_inv1,true, 0.05);
  Eigen::MatrixXd Z2 = J2*V1*VMV_inv1*V1.transpose();
  Eigen::MatrixXd ZMZ_inv2;
  pseudoInverse((Z2*mass*Z2.transpose()), ZMZ_inv2,true, 0.01);
  Eigen::MatrixXd J1_line = J1;
  Eigen::MatrixXd J2_line =ZMZ_inv2*Z2*mass;
  Eigen::MatrixXd Lambda1, Lambda2;
  Eigen::MatrixXd M_inv;
  pseudoInverse((mass), M_inv);
  pseudoInverse((J1*M_inv*J1.transpose()), Lambda1);
  Lambda2=Z2*mass*Z2.transpose();
  Eigen::MatrixXd Z1 = (M_inv*J1.transpose()*Lambda1).transpose();
  Eigen::MatrixXd nu_1 = J1_line*dq;
  Eigen::MatrixXd nu_2 = J2_line*dq; 
  d_x2_bar = J2*Z2.transpose()*nu_2;
  x2_bar = x2_bar+d_x2_bar*period.toSec();
  d_varphi2 = J2*(Z1.transpose()*nu_1);
  varphi2=varphi2+d_varphi2*period.toSec();
  Eigen::Matrix<double, 1, 1> x2_d;
  x2_d<< 0.25; //q_init_array[task2_j1]+0.1*sin(0.3*t.toSec());
  Eigen::Matrix<double, 1, 1> err_x2_bar=x2_bar-x2_d;
  Eigen::Matrix<double, 1, 1> err_x2 = x2-x2_d;
  Eigen::MatrixXd omega2 = -(K2*varphi2+D2*d_varphi2+Kt2*err_x2)/xt_2;

  if(T2<T_min2)
  {
    omega2=0.0*omega2;
  }
  
  double sigma2=0.5;
  double b2=0.000001;

  Eigen::Matrix<double,1,1> dxt2=(d_x2_bar.transpose()*D2*d_x2_bar);
  dxt2[0]=(sigma2*xt_2)/(xt_2*xt_2+T_min2)*dxt2[0];
  dxt2=-omega2.transpose()*d_x2_bar+dxt2;
  //barrier function
  dxt2[0]=dxt2[0]-b2*xt_2/(T_max2-0.5*xt_2*xt_2);
  xt_2=xt_2+dxt2[0]*period.toSec();
  T2=0.5*xt_2*xt_2;


  Eigen::Matrix<double, 7,7> Lambda;
  //Lambda.resize(7,7);
  //Lambda.setZero();
  Lambda <<Lambda1,Eigen::MatrixXd::Zero(6,1),
          Eigen::MatrixXd::Zero(1,6), Lambda2;

  Eigen::Matrix<double, 7, 7> J_line;
  J_line << J1_line, J2_line;
  Eigen::MatrixXd J_line_inv;
  pseudoInverse(J_line,J_line_inv);
  if(flag)
  {
   d_J_line=J_line/0.001;
   flag=false;
   J_line_last=J_line;
  }
  else
  {
   d_J_line=(J_line-J_line_last)/0.001;
   J_line_last=J_line;
  }
  Eigen::Matrix<double,7,7> miu;
  miu<< Lambda*(J_line*M_inv*C-d_J_line)*J_line_inv;
  Eigen::MatrixXd  miu11, miu12, miu21, miu22;
  miu11= miu.block(0, 0, Lambda1.rows(), Lambda1.cols());
  miu22= miu.block(Lambda1.rows(), Lambda1.cols(), Lambda2.rows(), Lambda2.cols());
  miu12= miu.block(0, Lambda1.cols(), Lambda1.rows(), Lambda2.cols());
  miu21= miu.block(Lambda1.rows(), 0, Lambda2.rows(), Lambda1.cols());
  taumiu= J1_line.transpose()*(miu12*nu_2)+ J2_line.transpose()*(miu21*nu_1);

  Eigen::Matrix7d B;
  Eigen::MatrixXd J2_aug_inv;
  B.setZero();
  pseudoInverse(J2_aug,J2_aug_inv);
  B=J_line*J2_aug_inv;
  Eigen::Matrix7d B_dot;
  B_dot.setZero();
  B_dot << (B-B_last)/0.001;
  B_last << B;
  Eigen::MatrixXd  B21, B21_dot, Gamma21, Phi21;
  B21 = B.block(Lambda1.rows(), 0 ,Lambda2.rows(), Lambda1.cols());
  B21_dot = B_dot.block(Lambda1.rows(), 0 ,Lambda2.rows(), Lambda1.cols());
  Gamma21=miu22*B21+Lambda2*B21_dot;
  Phi21=Lambda2*B21;
  Eigen::Matrix<double, 6, 1> d_x_aug;
  Eigen::Matrix<double, 6, 1> dd_x_aug;
  d_x_aug.setZero();
  dd_x_aug.setZero();
  d_x_aug=J1*dq;
  Eigen::Vector7d ddq=(dq-last_dq)/0.001;
  last_dq=dq;
  dd_x_aug=J1*ddq+S1_dot*dq;
  Eigen::Matrix<double,12,1> x_aug; 
  Eigen::Matrix<double,1,12> Gamma;
  Gamma.setZero();
  Gamma << Gamma21, Phi21;
  x_aug.setZero();
  x_aug << d_x_aug,dd_x_aug;
  taugamma= Gamma*x_aug;
  //std::cout <<"taugamma: "<<taugamma<<endl;
  // /* nullspace */
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

  // else
  // {
  //   rho_imp=0.5*(1+cos( (abs(error_ee[2])-d_max)/(d_max-d_min) ));
  //   ROS_INFO("rho_imp: %f", rho_imp);
  // }
  rho_frc=rho_imp;
  

  // if(sigma_imp<=error_ee[2])
  // {
  //   rho_imp=1.0;
  // }
  // else if(error_ee[2]>=0.0&&error_ee[2]<sigma_imp)
  // {
  //   rho_imp=0.5*(1-cos(M_PI*error_ee[2]/sigma_imp));
  // }
  // else
  // {
  //   rho_imp=0.0;
  // }
  // if((2*sigma_frc)<=abs(error_ee[2]))
  // {
  //   rho_frc=0.0;
  // }
  // else if(abs(error_ee[2])>=sigma_frc&&abs(error_ee[2])<2*sigma_frc)
  // {
  //   rho_frc=0.5*(1+cos(M_PI*(abs(error_ee[2])/sigma_imp-1)));
  // }
  // else
  // {
  //   rho_frc=1.0;
  // }
  /*更新f_end和Stiffiness*/
  //Stiffiness_end(2,2)=rho_imp*Stiffiness_end(2,2);
  //std::cout<<"Stiffiness_end: "<<endl<<Stiffiness_end<<endl;
  
  Stiffiness<<R_imp*Stiffiness_end*R_imp.transpose();
  kt<<rho_imp*Stiffiness;
  
  
  // Stiffiness_end=rho_imp*Stiffiness_end;
   //kt<<Stiffiness_end;
  
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
  //  //更新rho_imp和rho_frc, 以末端力误差
  // double d_max=3;
  // double d_min=1.5;
  // if(abs((f_end-force)[2])> d_max)
  // {
  //   rho_imp=0.0;
    
  // }
  // else if(abs((f_end-force)[2])< d_min)
  // {
  //   rho_imp=1.0;
  // }
  // else
  // {
  //   rho_imp=0.5*(1+cos( (abs((f_end-force)[2])-d_min)/(d_max-d_min)*M_1_PI ));
  //   ROS_INFO("rho_imp: %f", rho_imp);
  // }
  // rho_frc=rho_imp;



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
  // if(tank<= tank_max)
  // {
  //   beta = 1.0;
  // }
  // else
  // {
  //   beta = 0.0;
  // }
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

  // Eigen::MatrixXd jacobian_transpose_pinv;
  // pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  Eigen::Matrix<double, 7, 1> tau1 =J1_line.transpose()*(Lambda1*dvelocity_d+miu11*velocity_d-Damping*error_velocity-kc*error+omega*xt);
  //Eigen::Matrix<double, 7, 1> tau2 =(Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv)*J2.transpose()*(Lambda2*ddqd7+miu22*dqd7-D2*derr7-K2*err7+taugamma);
  //Eigen::Matrix<double, 7, 1> tau2 =J2_line.transpose()*Z2*J2.transpose()*(Lambda2*ddqd7+miu22*dqd7-D2*derr7-K2*err7+taugamma);
  
  Eigen::Matrix<double, 7, 1> tau2  = - J2_line.transpose()*Z2*J2.transpose()*( K2* 1.0* err_x2_bar+ D2* d_x2_bar-omega2*xt_2);
  
  //Eigen::Matrix<double, 7, 1> tau2 =J2.transpose()*(Lambda2*ddqd7+miu22*dqd7-D2*derr7-K2*err7+taugamma);
  //std::cout<<"tau2: "<<tau2<<endl;
  Eigen::Matrix<double, 7, 1>tau_sum=tau1+tau2+taumiu;
  //Eigen::Matrix<double, 7, 1>tau_sum=tau1;
  tau_cmd=tau_sum;
  
  double sigma=0.9;
  double b_factor=0.000001;
  Eigen::MatrixXd ut = -omega.transpose()*error_velocity;
  dxt=(sigma*xt)/(xt*xt+tank_min)*(error_velocity.transpose()*Damping*error_velocity)+ ut;  
  //barrier function
  dxt[0]=dxt[0]-b_factor*xt/(tank_max-0.5*xt*xt);
//barrier function
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
    robot.tau1[i]=tau1[i];
    robot.tau2[i]=tau2[i];
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

  // Eigen::Vector3d euler_d =transform.linear()*orientation_d_.toRotationMatrix().eulerAngles(0,1,2); //z y x
  // Eigen::Vector3d euler =transform.linear()*orientation.toRotationMatrix().eulerAngles(0,1,2); //z y x
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

  // robot.alphad=orientation_d_.x();
  // robot.alpha=orientation.x();
  // robot.alphaerr=orientation.x()-orientation_d_.x();
  // robot.betad=orientation_d_.y();
  // robot.beta=orientation.y();
  // robot.betaerr=orientation.y()-orientation_d_.y();
  // robot.gammad=orientation_d_.z();
  // robot.gamma=orientation.z();
  // robot.gammaerr=orientation.z()-orientation_d_.z();
  robot.fd=f_end[2];
  robot.fext=force[2];
  robot.ferr=force[2]-f_end[2];
  robot.q7=x2[0];
  robot.qd7=x2_d[0];
  robot.err7=err_x2[0];
  robot.tank=tank;
  robot.dxt=dxt[0];
  robot.rho_frc=rho_frc;
  robot.rho_imp=rho_imp;
  robot.error_ee=error_ee[2];
  robot.s=(0.5*error_velocity.transpose()*Lambda1*error_velocity+0.3*error.transpose()*kc*error)[0]+tank;
  Eigen::MatrixXd s2 = 0.5*(nu_2.transpose()*Lambda2*nu_2)+ 0.5*(err_x2_bar.transpose()*K2*err_x2_bar);
  robot.tank2=T2;
  robot.dxt2=dxt2[0];
  robot.s2=s2(0,0)+T2;
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

double UnifiedController::addValue(double value){
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
PLUGINLIB_EXPORT_CLASS(unified_controllers::UnifiedController,
                       controller_interface::ControllerBase)
