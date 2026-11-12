#include "../include/unified_power.h"
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
bool PowerUnifiedController::init(hardware_interface::RobotHW* robot_hw,
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

/**
 * @brief 控制器启动函数
 * @param time 当前ROS时间
 *
 * 此函数在控制器启动时被调用一次，用于初始化控制器状态、参数、ROS发布者等。
 */
void PowerUnifiedController::starting(const ros::Time& /*time*/) {
  // 创建一个新的ROS节点句柄，专门用于发布者
  ros::NodeHandle nhpub;
  // 初始化一个发布者，用于发布自定义的机器人状态消息到"/robotNewm"话题，队列大小为10
  pub =nhpub.advertise<unified_controllers::unified_robot>("/robotNewm",10);
  
  // 获取机器人启动时的完整状态
  franka::RobotState robot_state = state_handle_->getRobotState(); //这个是获取机械臂状态的
  
  // 初始化关节位置、速度和加速度数组
  q_init_array= robot_state.q;
  dq_init_array=robot_state.dq;
  ddq_init_array={0,0,0,0,0,0,0}; //开始加速度设为0
  
  // 将期望的关节状态初始化为当前状态，以确保平滑启动，避免初始跳变
  qd_array=q_init_array;
  dqd_array=dq_init_array;
  ddqd_array=ddq_init_array;
  
  // 初始化用于存储上一时刻关节速度的变量
  last_dq_array=dq_init_array;
  // 使用Eigen::Map高效地将数组映射为Eigen向量，无需数据拷贝
  Eigen::Map<Eigen::Matrix<double, 7, 1>> last_dq(last_dq_array.data()); 
  
  // 初始化时间和计数器
  t=ros::Duration(0.0);
  count=0;

  //--- 第一级任务（通常是末端执行器任务）的参数初始化 ---
  // 将初始关节位置数组映射为Eigen向量
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(robot_state.q.data());
  // 获取基坐标系到末端执行器的初始变换矩阵
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  //-------提取末端执行器的初始的位姿----------
  position_d_ = initial_transform.translation();//translation提取平移量
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());//linear提取旋转矩阵
  
  // 存储初始位置坐标，可能用于后续的轨迹生成
  a_circle=position_d_[0];
  b_circle=position_d_[1];
  c_circle=position_d_[2];
  
  // 获取初始的末端估计外力
  Eigen::Map<Eigen::Matrix<double, 6, 1>> f_initial(robot_state.O_F_ext_hat_K.data());
  
  // 初始化力控制相关的变量
  fe_last=f_initial; // 上一时刻的实际力
  f_init=f_initial;  // 初始力
  f_end_last.setZero(); // 上一时刻的期望力
  dfd.setZero(); // 期望力的导数
  dfe.setZero(); // 实际力的导数
  fd.setZero();  // 期望力
  f_end.setZero(); // 最终期望力
  f_end_target=6.0; // 设定Z轴方向的目标接触力为6N
  
  // 初始化关节误差向量
  joint_err.setZero();
  // 获取并存储初始的雅可比矩阵，用于后续计算其导数
  jacobian_old_array=model_handle_->getZeroJacobian(franka::Frame::kEndEffector);//kEndEffector末端执行器坐标系


  //--- 控制器增益和参数初始化 ---
  // 将所有控制器相关的矩阵初始化为零
  kt.setZero(); // 可变刚度矩阵
  kc.setZero(); // 固定刚度矩阵
  kp.setZero(); // 力控制P增益
  kd.setZero(); // 力控制D增益
  ki.setZero(); // 力控制I增益
  kp_target.setZero(); // 目标P增益
  ki_target.setZero(); // 目标I增益
  kd_target.setZero(); // 目标D增益
  Damping.setZero(); // 阻尼矩阵
  Stiffiness.setZero(); // 刚度矩阵（基坐标系）
  Stiffiness_end.setZero(); // 刚度矩阵（末端坐标系）

  // 设定力控制器PI增益的目标值（主要作用在Z轴）六个参数分别表示沿X,Y,Z轴的力和绕X,Y,Z轴的力
  kp_target.diagonal()<<0.0, 0.0, 1.25, 0.0, 0.0, 0.0;
  ki_target.diagonal()<<0.0, 0.0, 4.0, 0.0, 0.0, 0.0;
  kd_target.diagonal()<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  
  // 设定固定刚度矩阵 kc 的对角线元素
  kc.diagonal()<<1000.0,1000,0,1000.0,10.0,10.0,10.0;
  
  // 设定在末端坐标系下的期望刚度矩阵，变刚度的实现由缩放因子实现，即rho_imp
  Stiffiness_end.diagonal()<<800.0, 1500.0, 1000.0, 200.0, 200.0, 200.0;
  // 设定阻抗控制的阻尼矩阵
  Damping.diagonal()<<20.0, 20.0, 20.0, 5.0, 5.0, 5.0;

  // 初始化可能用于第二级任务的增益参数
  K2<<50.0;
  D2<<10.0;
  Kt2 << 100.0;

  // 初始化力控制的积分项
  fi.setZero();
  fi[2]=5.0; // 给Z轴的积分项一个初始值，可能有助于更快地达到目标力

  //--- 能量罐（Energy Tank）相关参数初始化 ---
  tank=0.5*xt*xt; // 基于状态xt计算初始能量
  
  // 初始化能量罐状态的导数
  dxt.setZero();
  // 初始化能量罐相关的调节因子
  alpha=1.0;
  gamma=0.5;

  // 初始化其他可能用到的变量
  B_last.setZero();
  x2_bar <<q_init_array[task2_j1]; // 使用特定关节的初始位置初始化第二级任务变量
  
}


void PowerUnifiedController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  //ROS_INFO("执行UnifiedController");
// 更新内部时间和计数器，用于轨迹生成和消息发布的频率控制
  count+=period.toSec();
  t+= period; 
  
  // 从state_handle_获取机器人当前时刻的完整状态，这是一个包含所有传感器信息的结构体
  franka::RobotState robot_state = state_handle_->getRobotState();
  
  // 从robot_state中提取出各个具体的状态量，并存储在C++标准数组中
  q_array=robot_state.q;      // 当前的关节角度 (7x1)
  dq_array=robot_state.dq;    // 当前的关节速度 (7x1)
  
  // 从model_handle_获取机器人当前的动力学模型参数
  mass_array = model_handle_->getMass();                  // 质量矩阵 M(q) (7x7)
  coriolis_array = model_handle_->getCoriolis();          // 科氏力/离心力向量 C(q, dq) * dq (7x1)
  gravity_array = model_handle_->getGravity();            // 重力向量 G(q) (7x1)
  // 获取末端执行器（EndEffector）参考坐标系下的雅可比矩阵，它是一个42元素的数组（代表6x7矩阵）
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // 下面的代码的作用是创建Eigen库中的映射（map）对象，将一个已存在的数据数组映射为Eigen矩阵或向量。
  // 这种操作非常高效，因为它不涉及内存拷贝，而是直接在原始数据上创建一个“视图”。
  // mass_array.data()用来获取mass_array容器的底层数据指针。
  Eigen::Map<Eigen::Matrix<double,7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data()); 
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());  
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double,7, 1>> q(q_array.data());
  Eigen::Map<Eigen::Matrix<double,7, 1>> dq(dq_array.data());
  
  // 将4x4的齐次变换矩阵（O_T_EE，从基坐标系到末端执行器坐标系的变换）映射并构建为一个Eigen::Affine3d对象，方便进行位姿操作
  //Eigen::Map()为在一块已经存在的，由裸指针指定的内存上，创建一个Eigen矩阵的视图或别名，而不进行任何数据的复制。
  //Matrix4d表示映射后的内容为一个4*4的矩阵。
  //Eigen::Affine3d专门用来表示三维空间仿射变换的类，其内部清晰地分离了旋转部分和平移部分。
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));//直接初始化，类的直接初始化
  // 从变换中提取平移部分，即当前末端执行器的位置向量
  Eigen::Vector3d position(transform.translation());
  // 从变换中提取旋转部分，并表示为四元数
  Eigen::Quaterniond orientation(transform.linear());
  
  // 再次映射齐次变换矩阵，这次是为了方便地提取旋转矩阵
  Eigen::Matrix4d Map1(robot_state.O_T_EE.data());
  // 从4x4矩阵的左上角提取3x3的块，即为旋转矩阵R
  Eigen::Matrix<double,3,3>R = Map1.block(0,0,3,3);//block(起始行索引，起始列索引，提取的行数，提取的列数)
  
  // 映射其他测量值
  // 当前测量到的关节力矩
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  // 在末端执行器坐标系下估计的外部扳力（力和力矩）
  Eigen::Map<Eigen::Matrix<double,6, 1>> f_ext(robot_state.O_F_ext_hat_K.data());//获取执行器末端的力
  // 经过滤波的外部关节力矩估计值
  Eigen::Map<Eigen::Matrix<double,7, 1>> tau_extm(robot_state.tau_ext_hat_filtered.data());
  
  // 调用一个自定义函数来计算科氏力矩阵C(q, dq)，注意这与libfranka提供的科氏力向量不同
  Eigen::Matrix7d C =CoriolisMatrix(q,dq);//科式力为虚拟力，无物理意义，其产生原因主要为旋转坐标系的旋转。
  
  // 映射上一控制周期的雅可比矩阵，用于计算其时间导数
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_old_array.data()); 
  
  // 声明用于存储雅可比导数的变量
  Eigen::Matrix<double,6,7> jacobian_dot;
  Eigen::Matrix<double,6,7> jacobian_dot_new;
  double r1 =0.01;

  //计算雅可比矩阵时
  if(firstUpdate)
  {
    S1=jacobian;
    jacobian_dot_new.setZero();
    S1_dot.setZero();
  }
  else
  {
    //S1_new = (period.toSec() / r1) * jacobian + (1 - period.toSec() / r1) * S1_old
    //离散滤波器
    jacobian_dot_new =(jacobian-jacobian_old)/period.toSec();
    S1_dot=(jacobian -S1)/r1;
    S1=S1_dot *period.toSec()+S1;
  }
  jacobian_old_array=jacobian_array;
  firstUpdate=false;

/****************************************************************************************************************************************************/
/*平面*/
  // double x=a_circle+0.1*sin(0.2*t.toSec());
  // double x=a_circle;
  // double y=b_circle+0.2*sin(0.15*t.toSec());
  // // double y=b_circle;
  // double z=c_circle;

  // // double dx =0.2*0.1*cos(0.2*t.toSec());
  // double dx =0.0;
  // double dy =0.15*0.2*cos(0.15*t.toSec());
  // // double dy =0.0;
  // double dz =0.0;

  // // double ddx =0.2*0.2*0.1*(-sin(0.2*t.toSec()));
  // double ddx =0.0;
  // double ddy =0.15*0.2*0.15*(-sin(0.15*t.toSec()));
  // // double ddy =0.0;
  // double ddz =0.0;
/****************************************************************************************************************************************************/

double x=a_circle;
double y=b_circle;
// double y=b_circle;
double z=c_circle;

double dx =0.0;
double dy =0.0;
// double dy =0.0;
double dz =0.0;
double ddx =0.0;
double ddy =0.0;
// double ddy =0.0;
double ddz =0.0;

/****************************************************************************************************************************************************/

  // --- 步骤 5: 设定期望轨迹并计算误差 ---
  // 将期望的位置、速度和加速度（在本例中为静态值）赋值给相应的Eigen向量
  position_d_<< x,y,z;  
  velocity_d <<dx, dy,dz,0.0,0.0,0.0;     // 期望速度向量（线速度+角速度）
  dvelocity_d <<ddx, ddy,ddz,0.0,0.0,0.0; // 期望加速度向量（线加速度+角加速度）
  
  // 通过雅可比矩阵和关节速度计算当前末端执行器的速度
  velocity<<jacobian*dq;
  // 计算位置误差（当前位置 - 期望位置）
  error.head(3) << position - position_d_;
  // 计算速度误差（当前速度 - 期望速度）
  error_velocity <<velocity - velocity_d ;

  // --- 姿态误差计算（使用四元数） ---
  // 检查当前姿态四元数和期望姿态四元数的点积。
  // 因为q和-q表示同一个旋转，这个检查确保我们总是计算最短的旋转路径，避免不必要的“绕远路”旋转。
  if (orientation.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // 计算从当前姿态到期望姿态的“差”四元数。公式为：q_error = q_current^-1 * q_desired，orientation.inverse()表示逆旋转，即如果orientation代表从基坐标系到当前坐标系，则它的逆表示从当前坐标系到基坐标系的旋转。
  //A.inverse()*B的数学意义是从姿态A到姿态B的相对旋转。
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  // 将上一步计算出的四元数误差，近似为一个三维的误差向量，并赋值给error向量的后三个元素（姿态对应的元素）。四元数的虚部向量（x,y,z）可以很好的近似这个旋转的轴-角表示，这个向量的方向近似于旋转轴，其模的大小与旋转角度的一半成正比。
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // 将计算出的姿态误差从末端执行器坐标系转换到基坐标系，以保证所有控制量在同一坐标系下计算
  // 这里是将末端执行器坐标系下的旋转向量转换到基坐标系下。
  error.tail(3) << -transform.linear() * error.tail(3);


  // --- 步骤 6: 更新控制器参数（平滑过渡）---
  // 定义一个非常小的滤波常数，用于非常缓慢、平滑地更新参数
  double filter_params_=0.001;
  // 使用一阶低通滤波器（指数移动平均）来平滑地将当前的PID增益（kp, ki, kd）过渡到目标值（kp_target等）。
  // 这样做可以防止因增益突变导致的系统震荡。公式：current = alpha * target + (1-alpha) * current
  kp.diagonal() = filter_params_ * kp_target.diagonal() + (1.0 - filter_params_) * kp.diagonal();
  ki.diagonal() = filter_params_ * ki_target.diagonal() + (1.0 - filter_params_) * ki.diagonal();
  kd.diagonal() = filter_params_ * kd_target.diagonal() + (1.0 - filter_params_) * kd.diagonal();

  // 同样地，平滑更新Z轴方向的目标力
  f_end[2]=filter_params_ * f_end_target + (1.0 - filter_params_)*f_end[2];
  
  // 定义一个相对较大的滤波常数，用于对传感器信号进行滤波
  double filter_params_1 =0.1;

  // 对测量到的外部力f_ext应用低通滤波器，以减少传感器噪声，获得更稳定的力读数
  f_ext=filter_params_1 * fe_last + (1.0 - filter_params_1) * f_ext;


/****************************************************************************************************************************************************/
// --- 步骤 7: 计算操作空间动力学参数 ---
  // 本节代码的目的是计算机器人动力学在“操作空间”（也叫“任务空间”或“笛卡尔空间”）中的等效表示。
  // 这对于实现如操作空间阻抗控制等高级控制器至关重要。

  // 声明一个6x7的矩阵J1，用于存储雅可比矩阵的副本。
  Eigen::Matrix<double, 6, 7> J1;
  // 将从机器人状态中获取的雅可比矩阵 'jacobian' 赋值给 J1。
  J1=jacobian;
  // 创建一个动态大小的矩阵J1_line，并用J1初始化。这可能是为了方便传递给某些需要动态矩阵作为参数的函数。
  Eigen::MatrixXd J1_line = J1;

  // 声明一个动态大小的矩阵Lambda1，它将用于存储操作空间（任务空间）的惯性矩阵。
  // 这个矩阵描述了末端执行器在笛卡尔空间中“感受”到的惯性。
  Eigen::MatrixXd Lambda1;
  // 声明一个动态大小的矩阵M_inv，用于存储关节空间惯性矩阵的逆。
  Eigen::MatrixXd M_inv;

  // 调用一个自定义函数 pseudoInverse 来计算关节空间惯性矩阵(mass)的（伪）逆，并将结果存入 M_inv。
  // 对于可逆的方阵（如此处的惯性矩阵），伪逆等于常规的逆。所以 M_inv = mass⁻¹。
  // 计算伪逆
  pseudoInverse((mass), M_inv);

  // 这是计算操作空间惯性矩阵Λ (Lambda) 的核心步骤。
  // 公式为： Λ = (J * M⁻¹ * Jᵀ)⁻¹
  // 首先计算括号内的部分 (J * M⁻¹ * Jᵀ)，它代表了关节惯性在操作空间的映射。
  // 然后对这个结果再次调用 pseudoInverse 函数求逆，得到最终的操作空间惯性矩阵Lambda1。
  pseudoInverse((J1*M_inv*J1.transpose()), Lambda1);

  // 声明一个6x6的矩阵miu。在操作空间动力学中，μ (mu) 通常用来表示
  // 作用在末端执行器上的等效科里奥利力/离心力矩阵。
  Eigen::Matrix<double,6,6> miu;
  
  /****************************************************************************************************************************************************/
  // --- 步骤 8: 变阻抗控制逻辑 ---
  // 本节代码实现了变阻抗控制的核心部分，包括坐标变换、基于误差的逻辑切换以及动态调整刚度。
  // 声明一个6x6的旋转矩阵R_imp，用于将扳力(wrench)或误差(error)从基坐标系转换到末端执行器坐标系。
  Eigen::Matrix<double,6,6> R_imp;
  // 初始化为零矩阵。
  R_imp.setZero();
  // 构建块矩阵。对角线上的 R.transpose() 分别用于旋转线性和角性分量。
  // R 是从基坐标系到末端坐标系的旋转矩阵，因此 R.transpose() 是从末端坐标系到基坐标系的旋转。
  // 注意：这里的构建方式意味着 R_imp 将会把一个在末端坐标系下定义的量转换到基坐标系。
  R_imp <<R.transpose(),Eigen::MatrixXd::Zero(3,3),
          Eigen::MatrixXd::Zero(3,3),R.transpose();
  
  // 使用 R_imp.transpose() 将在“基坐标系”下计算的误差向量 'error' 转换到“末端执行器坐标系”下。
  // 转换后的误差存储在 error_ee (error end-effector) 中。
  error_ee << R_imp.transpose()*error; //在末端坐标系下的误差
  
  // 对末端坐标系下的Z轴误差取反。这通常是为了符合任务的直观定义，
  // 例如，让“向下”的接触方向为正方向。
  error_ee[2]=-error_ee[2];

  // --- 基于末端位置误差，更新混合/插值因子 rho ---
  // 定义误差的上下阈值，用于创建一个“过渡区”。
  double d_max=0.025; // 误差大于该值时，认为处于非接触或大误差状态
  double d_min=0.015; // 误差小于该值时，认为处于稳定的接触/交互状态

  // 根据当前末端Z轴误差的大小，来计算混合因子 rho_imp。
  if(abs(error_ee[2])> d_max)
  {
    // 如果误差大于最大阈值，则将混合因子设为0，意味着完全禁用变刚度和力控制。
    rho_imp=0.0;
    // 同时重置力控制的积分项，防止在非接触时积分项过度累积（积分饱和）。
    fi[2]=5.0;
  }
  else if(abs(error_ee[2])< d_min)
  {
    // 如果误差小于最小阈值，则将混合因子设为1，意味着完全启用变刚度和力控制。
    rho_imp=1.0;
  }
  else
  {
    // 如果误差处于过渡区内，则使用一个平滑的余弦函数来计算混合因子，
    // 使其从1（在d_min处）平滑地过渡到0（在d_max处）。
    rho_imp=0.5*(1-cos( abs((error_ee[2])-d_max)/(d_max-d_min)*M_1_PI ));
  }
  // 让力控制的混合因子 rho_frc 与阻抗控制的混合因子 rho_imp 保持一致。
  rho_frc=rho_imp;
  
  // --- 计算最终应用的刚度矩阵 ---
  // 使用旋转矩阵 R_imp 将在“末端坐标系”下定义的期望刚度 'Stiffiness_end' 转换到“基坐标系”下。
  // 这是张量旋转的标准公式： K_base = R * K_end_effector * R^T
  Stiffiness<<R_imp*Stiffiness_end*R_imp.transpose();
  // 将转换到基坐标系的刚度矩阵 Stiffiness 与混合因子 rho_imp 相乘，
  // 得到当前时刻实际应用的、随误差动态变化的可变刚度矩阵 kt。
  kt<<rho_imp*Stiffiness;
  
  

// --- 步骤 9: 力控制器与能量罐逻辑 ---

  // 声明一个6x6的旋转矩阵R_frc，用于将扳力(wrench)从基坐标系转换到末端执行器坐标系。
  Eigen::Matrix<double,6,6> R_frc;
  // 初始化为零矩阵。
  R_frc.setZero();
  // 构建块矩阵。对角线上的R矩阵分别用于旋转线性和角性分量。
  // R是从基坐标系到末端坐标系的3x3旋转矩阵。
  R_frc <<R,Eigen::MatrixXd::Zero(3,3),
          Eigen::MatrixXd::Zero(3,3),R;

  // 使用后向差分法（数值微分）计算期望力f_end的时间导数。
  dfd=(f_end-f_end_last)/0.001;
  // 将测量到的外部力f_ext（在基坐标系下）转换到末端执行器坐标系，并提取Z轴分量。
  double force_value_change = (R_frc*f_ext)[2];
  // 对提取出的力值应用滑动平均滤波器以减少噪声（注意：下一行代码会覆盖这个结果）。
  double force_value = addValue(force_value_change);
  // 将未经滤波的值赋给force_value，实际未使用滤波结果。
  force_value=force_value_change;
  // 创建一个6x1的向量force，只填充Z轴方向的力值，代表当前测量到的接触力。
  Eigen::Matrix<double, 6, 1> force;
  force.setZero();
  force << 0.0, 0.0, force_value, 0.0, 0.0, 0.0;
  
  // 使用数值微分计算实际接触力force的时间导数。
  dfe=(force-fe_last)/0.001;
  // 更新上一时刻的期望力和实际力，为下一次迭代做准备。
  f_end_last=f_end;
  fe_last=force;

  // --- PID力控制器 ---
  // 计算积分项(I-term)：累积期望力与实际力之间的误差。
  fi=fi+ki*(f_end-force)*0.001;
  // 声明力控制器的输出向量。
  Eigen::Matrix<double, 6, 1> tau_force;
  Eigen::Matrix<double, 6, 1> tau_force_e; // 可能用于调试或数据记录的副本。
  tau_force.setZero();
  // 在末端执行器坐标系下，计算完整的PID控制输出。
  // 包括：期望力前馈(f_end)、比例项(P)、积分项(I)和微分项(D)。
  tau_force<< f_end+kp*(f_end-force)+ ki*fi+ kd*(dfd-dfe);  //在末端坐标系下
  
  // 使用之前计算的混合因子rho_frc来缩放PID控制器的输出。
  // 当机器人远离目标接触位置时，rho_frc接近0，力控制作用减弱。
  tau_force << rho_frc*tau_force;
  tau_force_e << tau_force; // 保存缩放后的值到副本。
  // 将计算出的扳力tau_force从末端执行器坐标系转换回基坐标系，以便与整个动力学模型统一。
  tau_force = R_frc.transpose()*tau_force; //转换到基坐标系


  // --- 能量罐（Energy Tank）调节逻辑 ---
  // 根据能量罐中当前的能量(tank)，计算一个调节因子alpha，用于保证系统的无源性。
  if(tank>=tank_min+delta_E)
  {
    // 如果能量充足，则alpha为1，允许能量交换。
    alpha=1.0;
  }
  else if(tank>=tank_min && tank<tank_min+delta_E)
  {
    // 如果能量处于一个过渡区间，则使用一个平滑的S型函数来计算alpha，使其在0和1之间平滑过渡。
    // 这可以防止系统行为的突变。
    double exp1=std::exp(1.0/(tank-tank_min));
    double exp2=std::exp(1.0/(tank_min+delta_E-tank));
    alpha=1.0/(1.0+exp1/exp2);
  }else{
    // 如果能量低于阈值，则alpha为0，禁止从系统中抽取能量。
    alpha=0.0;
  }

  // --- 计算能量交换项 omega ---
  // omega是控制器中用于实现能量交换的关键项。
  // 它结合了由可变刚度产生的虚拟力(-kt*error)和由力控制器产生的主动力(tau_force)。
  // 整个表达式通过alpha和能量罐状态xt进行缩放，从而将能量交换限制在无源性约束之内。
  omega = alpha/xt*(-kt*error+tau_force);



  // --- 步骤 10: 计算操作空间动力学矩阵 ---

  // 声明用于存储雅可比矩阵转置的伪逆的变量
  Eigen::MatrixXd jacobian_transpose_pinv;
  // 调用自定义函数计算雅可比矩阵转置的伪逆。'true'和'0.2'可能是阻尼最小二乘法的参数。
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv,true,0.2);
  // 声明用于存储雅可比矩阵伪逆的变量
  Eigen::MatrixXd jacobian_pinv;
  // 计算雅可比矩阵的伪逆。
  pseudoInverse(jacobian, jacobian_pinv,true,0.2);

  // 声明用于存储自定义动力学模型计算结果的变量。
  // 注意：这些似乎是重复计算，因为libfranka已经提供了这些值。这可能是为了与特定的理论模型进行比较或验证。
  Eigen::MatrixXd m_mario; 
  Eigen::MatrixXd c_mario; 
  Eigen:: MatrixXd g_mario; 
	m_mario=MassMatrix(q);
  c_mario=CoriolisMatrix(q,dq);
  g_mario=GravityVector(q);  
  // 计算质量矩阵的伪逆（对于可逆的质量矩阵，这等同于求逆）。
  Eigen::MatrixXd m_mario_pinv;
  pseudoInverse(mass, m_mario_pinv,true,0.2);

  // 使用后向差分法（数值微分）计算雅可比矩阵的时间导数 d_jacobian。
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
  
  double sigma=0.9;
  double b_factor=0.000001;
  Eigen::MatrixXd ut = -omega.transpose()*error_velocity;
  // dxt=(sigma*xt)/(xt*xt+tank_min)*(error_velocity.transpose()*Damping*error_velocity)+ ut;  
  dxt=xt/(xt*xt+tank_min)*(error_velocity.transpose()*Damping*error_velocity)+ alpha_t*ut;
  //barrier function
  dxt[0]=dxt[0]-b_factor*xt;
//barrier function
  xt=xt+dxt[0]*0.001;

  Eigen::MatrixXd P=xt*ut;
  double p_flow=P(0,0);
  tank= 0.5 *xt*xt;

  // 这是一个安全机制：如果计算出的功率流的绝对值超过预设的限制`P_limit`...
  if(abs(p_flow)>P_limit)
  {
    // ...则计算一个缩放因子`alpha_t`来将功率流限制在`P_limit`之内。
    alpha_t=P_limit/abs(p_flow);
  }else
  {
    // 否则，不进行缩放。
    alpha_t=1;
  }

// --- 步骤 13: 计算最终关节力矩并发送指令 ---

  // 这是核心的“操作空间动力学控制率”。它计算出一个在操作空间中期望施加的扳力(wrench)，然后通过雅可比矩阵的转置将其映射回关节空间的力矩。
  // 括号内的项包括：期望加速度前馈(Lambda1*dvelocity_d)，期望速度前馈/科氏力补偿(miu*velocity_d)，阻尼项(-Damping*error_velocity)，固定刚度项(-kc*error)，以及经过功率限制和能量罐调节的能量交换项(alpha_t*omega*xt)。
  Eigen::Matrix<double, 7, 1> tau1 =J1_line.transpose()*(Lambda1*dvelocity_d+miu*velocity_d-Damping*error_velocity-kc*error+alpha_t*omega*xt);//输出力矩
  tau_cmd=tau1; // （注意：为了获得更好的性能，通常在这里会加上重力补偿，如 tau_cmd = tau1 + gravity;）

// 获取上一周期发送的力矩指令，用于力矩变化率的限制。
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
// 调用安全函数，对计算出的力矩进行变化率和平滑处理，并限制其在安全范围内。
  tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);//力矩保护
 //std::cout<<"tau_cmd222: "<<endl<<tau_cmd<<endl;

 // 遍历所有关节，将最终计算出的安全力矩指令发送给机器人硬件。
 for(size_t i=0;i<7;i++)
  {
     //ROS_INFO("tau_cmd: %f", tau_cmd[i]);
    joint_handles_[i].setCommand(tau_cmd[i]); //发送力矩
    robot.tau[i]=tau_cmd[i];
    robot.tau1[i]=tau1[i];
    // robot.tau2[i]=tau2[i];
  }

  // --- 步骤 14: 填充ROS消息用于发布 ---
  // 将控制器内部的各种状态、误差、力和参数填充到一个自定义的ROS消息'robot'中，用于实时监控、绘图和调试。
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
  robot.alpha_t=alpha_t;
  robot.p_limit=P_limit;
  robot.p_valve=alpha_t*abs(p_flow);
  
 // --- 步骤 15: 定期发布消息 ---
  // 控制发布频率，以避免占用过多网络带宽。这里设置为大约10Hz (1/0.1s)。 
 if(count>0.1)
  {
    count=0.0;
    pub.publish(robot);} 
}

Eigen::Matrix<double, 7, 1> PowerUnifiedController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax); //控制力矩变化率
    //以下的力矩限制可以通过自己的需要更改，详见fci
   if(i==0||i==1||i==2||i==3)
    {
        if(tau_d_saturated[i]>35)
        {
            tau_d_saturated[i]=35;
        }
        if(tau_d_saturated[i]<-35)
        {
            tau_d_saturated[i]=-35;
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

double PowerUnifiedController::addValue(double value){
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
PLUGINLIB_EXPORT_CLASS(unified_controllers::PowerUnifiedController,
                       controller_interface::ControllerBase)
