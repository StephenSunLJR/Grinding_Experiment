// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#ifndef _FUZZY_EXAMPLE_CONTROLLER_H
#define _FUZZY_EXAMPLE_CONTROLLER_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
// #include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

using std::string;

namespace franka_example_controllers {

// class FuzzyExampleController : public controller_interface::MultiInterfaceController<
//                                                 franka_hw::FrankaModelInterface,
//                                                 hardware_interface::EffortJointInterface,
//                                                 franka_hw::FrankaStateInterface> {
    class FuzzyExampleController : public controller_interface::MultiInterfaceController<
                                        hardware_interface::PositionJointInterface>{
 public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

private:
    hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;

  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};

    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{20.0};
    const double delta_tau_max_{1.0};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    std::mutex position_and_orientation_d_target_mutex_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;

   std::array<double, 7> saturateTorqueRate(
            const std::array<double, 7>& tau_d_calculated,
            const std::array<double, 7>& tau_J_d);

    // std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    // std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    // std::vector<hardware_interface::JointHandle> joint_handles_;

    static constexpr double kDeltaTauMax{1.0};
    ros::Duration elapsed_time_;
    std::array<double, 7> initial_pose_{};
    std::array<double, 7> dpose_desired;
    std::array<double,7> integral{0};  

    std::vector<double> k_gains_{300,300,300,250,0,250,0};
    std::vector<double> d_gains_{10,10,10,10,10,20,10};
    std::vector<double> i_gains_{0,0,0,0,0,0,0};
    std::array<double, 7> dq_filtered_;

    public:
    const static int N=7;
    private:
    std::array<double,2> target{0}; //系统控制目标
    std::array<double,2> actual{0}; //采样获得的实际指
    std::array<double,2> e{0};       //误差
    std::array<double,2> e_pre_1{0}; //上一次的误差
    std::array<double,2> e_pre_2{0}; //上上次的误差
    std::array<double,2> de{0};     //误差变化率
    double emax=0;   //误差基本论域上限
    double demax=0;  //误差变化率基本论域上限
    float delta_Kp_max=0;  //delta_kp输出的上限
    float delta_Ki_max=0; //delta_ki输出上限
    float delta_Kd_max=0; //delta_kd输出上限
    float Ke=0;          //Ke=n/emax
    float Kde=0;         //Kde=n/emax
    float Ku_p=0;        //Ku_p=Kpmax/n
    float Ku_i=0;         //Ku_i=Kimax/n
    float Ku_d=0;         //Ku_d=kdmax/n
    int Kp_rule_matrix[N][N]{0}; //Kp模糊规则矩阵
    int Ki_rule_matrix[N][N]{0};//Ki模糊规则矩阵
    int Kd_rule_matrix[N][N]{0};//Kd模糊规则矩阵
    string mf_t_e="No Type";          //e的隶属度函数类型
    string mf_t_de="No Type";         //de的隶属度函数类型
    string mf_t_Kp="No Type";         //Kp的隶属度函数类型
    string mf_t_Ki="No Type";         //Ki的隶属度函数类型
    string mf_t_Kd="No Type";         //Kd的隶属度函数类型
    float *e_mf_paras=NULL;      //误差的隶属度函数的参数
    float *de_mf_paras=NULL;     //误差的变化率的隶属度函数的参数
    float *Kp_mf_paras=NULL;     //Kp的隶属度函数的参数
    float *Ki_mf_paras=NULL;     //Ki的的隶属度函数的参数
    float *Kd_mf_paras=NULL;     //Kd的的隶属度函数的参数

    std::array<double,2> Kp{0};
    std::array<double,2> Ki{0};
    std::array<double,2> Kd{0};

    std::array<double,7> A{0};
    std::array<double,7> B{0};
    std::array<double,7> C{0};
    std::array<double,7> fuzzy_e{0};
    std::array<double,7> fuzzy_e_pre_1{0};
    std::array<double,7> fuzzy_e_pre_2{0};
    void setMf_sub(const string & type,float *paras,int n);//设置模糊隶属度函数的子函数

public:
    void Init_FuzzyExampleController(double e_max,double de_max,float kp_max,float ki_max,float kd_max,float Kp0,float Ki0,float Kd0);
    // FuzzyPIDController(float *fuzzyLimit,float *pidInitVal);
    ~FuzzyExampleController();
    float trimf(float x,float a,float b,float c);          //三角隶属度函数
    float gaussmf(float x,float ave,float sigma);          //正态隶属度函数
    float trapmf(float x,float a,float b,float c,float d); //梯形隶属度函数
    void setMf(const string & mf_type_e,float *e_mf,
			   const string & mf_type_de,float *de_mf,
			   const string & mf_type_Kp,float *Kp_mf,
		       const string & mf_type_Ki,float *Ki_mf,
			   const string & mf_type_Kd,float *Kd_mf);	//设置模糊隶属度函数的参数
    void setRuleMatrix(int kp_m[N][N],int ki_m[N][N],int kd_m[N][N]);  //设置模糊规则
    void realize(float t,float a,int index);  //实现模糊控制
};

}  // namespace franka_example_controllers

#endif