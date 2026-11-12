#include "../include/icvs_energy.h"
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
bool icvs_energy::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::string arm_id;
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("arm_id", arm_id)) {
       ROS_ERROR("icvs_energy: Could not read parameter arm_id");
       return false;
     }
  
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "icvs_energy: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "icvs_energy: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "icvs_energy: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("icvs_energy: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "icvs_energy: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "icvs_energy: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "icvs_energy: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  if (!node_handle.getParam("sigma", sigma_) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no sigma_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("xt", xt_) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no xt_ parameters provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("kc", kc_) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no kc_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kt", kt_) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no kt_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("ktmax", kt_max) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no kt_max parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("ktmin", kt_min) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no kt_max parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("min_tank_level", min_tank_level_) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no min_tank_level parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("max_tank_level", max_tank_level_) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no max_tank_level parameters provided, aborting "
        "controller init!");
    return false;
  }
  
    if (!node_handle.getParam("lambda", lambda) ) {
    ROS_ERROR(
        "icvs_energy:  Invalid or no max_tank_level parameters provided, aborting "
        "controller init!");
    return false;
  }
  return true;
}

void icvs_energy::starting(const ros::Time& /*time*/) {
  ros::NodeHandle nhpub;
  pub =nhpub.advertise<variable_stiffness_controllers::compliance_robot>("/robotNew",10);
  sub = nhpub.subscribe<std_msgs::String>("emg_signal",100,&icvs_energy::doEmg,
                                                          this, ros::TransportHints().reliable().tcpNoDelay());//回调函数doEmg用来处理订阅到的数据


  franka::RobotState robot_state = state_handle_->getRobotState(); //这个是获取机械臂状态的
  q_init_array= {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};//robot_state.q;
  dq_init_array=robot_state.dq;
  ddq_init_array={0,0,0,0,0,0,0}; //开始加速度设为0

  qd_array=q_init_array;
  dqd_array=dq_init_array;
  ddqd_array=ddq_init_array;
  //初始值设置
  t=ros::Duration(0.0);
  // md.diagonal()<<md_gains_[0],md_gains_[1],md_gains_[2],md_gains_[3],md_gains_[4],md_gains_[5];
  // dd.diagonal()<<dd_gains_[0],dd_gains_[1],dd_gains_[2],dd_gains_[3],dd_gains_[4],dd_gains_[5];
  // kd.diagonal()<<kd_gains_[0],kd_gains_[1],kd_gains_[2],kd_gains_[3],kd_gains_[4],kd_gains_[5];
  md.setZero();
  dd.setZero();
  // kd.setZero();
  md.diagonal()<<16.0, 16.0, 16.0, 6.0, 6.0, 6.0;
  dd.diagonal()<<30.0, 30.0, 30.0, 20.0, 20.0, 20.0;
  // kd.diagonal()<<50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
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

  //设置末端目标姿态
  // Eigen::AngleAxisd rollAngle(0,Eigen::Vector3d::UnitX());
  // Eigen::AngleAxisd pitchAngle(0,Eigen::Vector3d::UnitY());
  // Eigen::AngleAxisd yawAngle(-0.7854,Eigen::Vector3d::UnitZ());
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  //orientation_d_ = yawAngle*pitchAngle*rollAngle;
  a_circle=position_d_[0];
  b_circle=position_d_[1];
  c_circle=position_d_[2];
  f_tau.setZero();
  jacobian_old_array=model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  wt_ << 0.1,0.1,0.1,0.1,0.1,0.1;
  tau_cmd.setZero();
  last_kt=kt_;
  
  // FILE* icvs_energy_date=fopen("/home/liu/franka_ros_test/catkin_ws/icvs_energy_date.text","w");
  // fprintf(icvs_energy_date,"t.toSec()\t,position1[0]\t,position1[1]\t,position1[2]\t,position_d_[0]\t,position_d_[1]\t,position_d_[2]\t,error[1]\t,error[2]\t,error[3]\t,tau_cmd[0]\t,tau_cmd[1]\t,tau_cmd[2]\t,tau_cmd[3]\t,tau_cmd[4]\t,tau_cmd[5]\t,tau_cmd[6]\t,max_tank_level_\t,T,robot.kd\t");
  // fclose(icvs_energy_date);
}

void icvs_energy::update(const ros::Time& /*time*/, const ros::Duration& period) {
  count+=period.toSec();
  t+= period; 
  franka::RobotState robot_state = state_handle_->getRobotState();
  q_array=robot_state.q;
  dq_array=robot_state.dq;
  tau_J_d_array=robot_state.tau_J_d;//获取上一个周期的期望力矩，供后续的平滑函数使用。
  
  mass_array = model_handle_->getMass();
  coriolis_array = model_handle_->getCoriolis();
  gravity_array = model_handle_->getGravity();
  //Eigen映射
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());//这里的q是模板类Eigen::Map的一个特化实例，Eigen::Matrix指定了我们希望q这个Map对象表现的像哪种具体的Eigen类型，即一个7行1列，元素类型为double的矩阵。
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(tau_J_d_array.data());//期望的关节力矩向量
  Eigen::Map<Eigen::Matrix<double,7, 7>> mass(mass_array.data());  //描述了7个关节之间的惯性耦合关系
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data()); //由关节运动产生的耦合力矩 
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());  //为抵抗重力，需要再每个关节上施加的力矩
  Eigen::Matrix<double, 7, 1> mass_of_cube ;
  mass_of_cube<<5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
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
  
  //--------轨迹生成开始----------

  double x=a_circle;//+radius_*cos(factor_circle*t.toSec());
  double y=b_circle;//+radius_*sin(factor_circle*t.toSec());
  double z=c_circle;//+radius_*cos(factor_circle*t.toSec());
  position_d_<<x,y,z;
  error.head(3) << position - position_d_;
  double dx=0.0;
  double dy=0;//factor_circle*radius_*cos(factor_circle*t.toSec());
  double dz=0;//-factor_circle*radius_*cos(factor_circle*t.toSec());

  double ddx=0.0;
  double ddy=0;//-factor_circle*factor_circle*radius_*sin(factor_circle*t.toSec());
  double ddz=0;//factor_circle*factor_circle*radius_*sin(factor_circle*t.toSec());
  velocity_d_<<dx,dy,dz,0,0,0; //角速度设置为0
  dvelocity_d_<<ddx,ddy,ddz,0,0,0; //角加速度设置为0
  error_velocity=jacobian * dq-velocity_d_;

  //---------轨迹生成结束----------

  //四元数相关运算
  // orientation error
  //用四元数的点乘计算期望和实际旋转之间的差值，
  //作用：用于插值，确保总是沿着最短的路径进行
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
  
  Eigen::MatrixXd jacobian_transpose_pinv;//雅可比伪逆的转置
  franka_interactive_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);//用后者储存计算得到的雅可比转置的伪逆
  Eigen::MatrixXd jacobian_pinv;//雅可比伪逆
  franka_interactive_controllers::pseudoInverse(jacobian, jacobian_pinv);//用后者储存计算得到的雅可比矩阵的伪逆

	MatrixXd m_mario; MatrixXd c_mario; MatrixXd g_mario; //惯性矩阵、科里奥利力、重力向量
	m_mario=MassMatrix(q);
  c_mario=CoriolisMatrix(q,dq);
  g_mario=GravityVector(q);  
  Eigen::Matrix<double,6,6>miu=jacobian_transpose_pinv*(c_mario*-m_mario*jacobian_pinv*S1_dot)*jacobian_pinv;//笛卡尔空间的科里奥利力
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

//下面内容如何得到的
  wt_=-(kt_matrix*error)/(xt_);
  
  double T=0.5*xt_*xt_;
  kt_=100;//220*sin(M_PI/10*t.toSec());
  kt_matrix.diagonal()<<kt_,kt_,kt_,0,0,0; 
  //wt_条件
  //防止能量罐中能量过少的策略
  if( T >min_tank_level_ )
  {
    dela_xt=0; 
  }
  else
  {
    dela_xt=min_tank_level_-sqrt(2*T*min_tank_level_-T*T);//能量小于最低阈值时，dela_xt从0平滑地开始增大，没有发生跳变
  } 

  //当能量低于最低阈值时，wt_的分母会平滑地增大，wt_平滑地减小，从而使控制指令在能量耗尽时平滑地衰减至零。
  wt_=-(kt_*xt_*error)/(xt_*xt_+dela_xt);//参照毕业论文第四章，该参数w与机械臂的变刚度相关，设计成公式这种形式是为了保证任务执行的平滑性。
  /*条件2*/

  
  /* sum energy tank with barrier function */
  // Eigen::MatrixXd dxt1_=(error_velocity.transpose() *dd* error_velocity)*(sigma_* xt_ /(xt_ *xt_+min_tank_level_));
  // dxt1_=dxt1_ - (wt_.transpose()*error_velocity);
  Eigen::MatrixXd dxt1_=((dd-a*Lambda-r)*error_velocity+(kc_matrix-a*miu-a*r)*error-wt_*xt_)*(error_velocity+a*error).transpose();
  // double b_factor =0.02;
  //障碍函数
  // dxt1_(0,0)= dxt1_(0,0) -lambda*xt_+b_factor*xt_/(T-min_tank_level_);
  dxt1_(0,0)= dxt1_(0,0) -lambda*xt_;//简化后的耗散项,-lambda*xt_这一项保证能量罐中的能量会随着时间主动“泄漏”，保证整个控制系统的被动性和稳定性

  //-----------------------------------------

  /* sum energy tank with barrier function */

  Eigen::Matrix<double,7 ,7> I_7x7;
  I_7x7.setIdentity();
  Eigen::MatrixXd jacobian_pinv_;
  franka_interactive_controllers::pseudoInverse(jacobian,jacobian_pinv_);
  Eigen::Matrix<double,7 ,7> N=I_7x7-jacobian_pinv_*jacobian;

  double dxt_=dxt1_(0,0);
  xt_=xt_+dxt_*period.toSec();//xt_ ：更新公式
  //sum energy tank
  f_tau= Lambda*dvelocity_d_+miu*velocity_d_- dd*error_velocity- kc_matrix*error;//+ wt_*xt_;//笛卡尔空间中的广义力，包含（x,y,z,roll,pitch,yaw）六个方向上的力

  // /* no energy tank */
  // f_tau= Lambda*dvelocity_d_+miu*velocity_d_- dd*error_velocity- (kc_+kt_)*error;
  // /* no energy tank */
  double K_null = 0;//零空间阻尼，待调整
  tau_cmd = jacobian.transpose() *f_tau ;//+ gravity; //tau=J.trans * F: 由笛卡尔空间的力转换到关节空间中的力 
  //tau_cmd = tau_cmd - K_null * N * dq;//施加与零空间速度相反的力矩
  tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);//力矩保护
  //std::cout<<"tau_cmd: "<<tau_cmd<<endl;
  for(size_t i=0;i<7;i++)
  {
     joint_handles_[i].setCommand(tau_cmd[i]); //发送力矩
     robot.taud[i]=tau_cmd[i];
  }
  ROS_ERROR_STREAM_THROTTLE(0.5, "tau_cmd: "<<tau_cmd);

//----------打包数据，并进行发布，供其他节点来订阅--------

  /*
    重新获取机械臂状态的目的：
    1.确保发布的数据是最新时刻的状态，而不是控制计算时使用的可能已经稍微滞后的数据
    2.提高系统的实时性与可靠性
    3.状态监测和数据记录的准确性
  */
  robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform1(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position1(transform1.translation());
  Eigen::Quaterniond orientation1(transform1.linear());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> f_ext(robot_state.O_F_ext_hat_K.data());//法兰盘上转换到世界坐标系下的力，（3个力，3个力矩共六维）
  Eigen::Matrix<double,1,1> change;
  change.setOnes();
  //sum energy tank  总能量：机器人动能+机器人弹性势能+能量罐中的能量
  Eigen::MatrixXd v=0.5*error_velocity.transpose()*Lambda*error_velocity+0.5*error.transpose()*(kc_matrix+kt_matrix)*error+0.5*xt_*xt_*change;//
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
  robot.sumerror= error.transpose()*error;//笛卡尔空间位姿误差向量的平方范数
  robot.t=t.toSec();
  for(size_t i=0;i<6;i++)
  {
    robot.positionerror[i]=error[i];
  }
  if(count>0.1)
  {
    count=0;
  pub.publish(robot);}
  //--------------------
}
//下面的回调函数在controller_manager中调用，用于处理来自其他节点的消息,以spin()函数的形式调用
//没有spin函数就很有可能订阅不到消息
//订阅节点创建处只是注册了回调函数，真正触发回调函数的函数是spin()函数
void icvs_energy::doEmg(const std_msgs::String::ConstPtr& msg_p){
  //msg_p为传入的消息引用，通过msg_p获取并操作订阅到的数据

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

Eigen::Matrix<double, 7, 1> icvs_energy::saturateTorqueRate(
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
PLUGINLIB_EXPORT_CLASS(variable_stiffness_controllers::icvs_energy,
                       controller_interface::ControllerBase)
