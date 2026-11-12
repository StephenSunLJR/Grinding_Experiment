#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include "variable_stiffness_controllers/compliance_robot.h"
#include "matrix_utils_for_interactive.h"
#include <std_msgs/Float64MultiArray.h>
//还需要添加一个msg文件对应的头文件333232323232

namespace interactive_force_impedence_control { 
    class interactive_control: public controller_interface::MultiInterfaceController<
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaModelInterface,
                                   franka_hw::FrankaStateInterface> { 
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        double calculate_damping(double S_E_f, double S_E_f_l, 
                         double S_delta_h, double S_delta_s, double p, double epsilon);
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d) ;
    private:
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; 
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;//机械臂关节节点
        double count{0.0};
        ros::Duration t;
        ros::Publisher pub;
        ros::Subscriber sub;
        double Pc;//C-space的交互功率
        double Pu;//U-space的交互功率
        //----力能量罐------
        double z_f;         //力能量罐的状态
        double T_E_f;       //力能量罐总能量
        double I_E_f;       //交互腔能量
        double P_f;         //流入能量罐的功率
        double lambda_c{0};    //阻尼调制系数
        double lamada_Pc{0};   //交互室能量变化速度控制参数
        double d_f_T{0};       //总能量衰减项
        double d_f_I{0};       //交互腔能量衰减项

        //----阻抗能量罐
        double z_i;         //阻抗能量罐的状态
        double T_E_i;       //阻抗能量罐总能量
        double I_E_i;       //交互腔能量
        double P_i;         //流入阻抗能量罐的功率
        double d_i_T{0};       //总能量衰减项
        double d_i_I{0};       //交互腔能量衰减项

        //---能量罐参数
        double P_Vf;        //力能量罐的阀门速率
        double P_Vi;        //阻抗能量罐的阀门速率
        double T_E_f_l;     //力能量罐总能量下限
        double I_E_f_l;     //力能量罐交互腔能量下限
        double T_E_i_l;     //阻抗能量罐总能量下限
        double I_E_i_l;     //阻抗能量罐交互腔能量下限


        typedef Eigen::Matrix<double,6,1> Vector6d;
        
        Vector6d position_dot;//实际速度
        Eigen::Vector3d position_d; //期望笛卡尔姿态
        Vector6d position_d_dot; //期望笛卡尔速度
        Vector6d position_d_ddot;//期望笛卡尔加速度
        Vector6d pose_tilde;//位置误差
        Vector6d pose_tilde_dot;//位置误差导数
        Eigen::Vector3d position_init;
        double radius_{0.15};
        double factor_circle{0.2};

        Vector6d F_ext{}; //外部力
        Vector6d Fd_F{};  //期望力
        Vector6d Fd_bin{};//力方向选择二进制向量
        Vector6d Fd_prime{};//修正后的期望力
        Vector6d Ff_i_d{};//力控的积分、微分项

        Vector6d Ff_prime{};//最终衰减后的力控输出
        Vector6d xd_dot_prime{};//最终衰减后的期望速度

        //-----6X6的矩阵

        Eigen::Matrix<double,6,6> Kp;
        Eigen::Matrix<double,6,6> Kd;
        Eigen::Matrix<double,6,6> Ki;
        Eigen::Matrix<double,6,6> R6_W_F_i;
        Eigen::Matrix<double,6,6> R6_W_F_w;

        Eigen::Matrix<double,6,6> K_s;//阻抗控制刚度
        Eigen::Matrix<double,6,6> D_d;//阻抗控制阻尼
        Eigen::Matrix<double,6,6> D_w;//力控方向矩阵（Dw）
        Eigen::Matrix<double,6,6> D_w_c;//C-space投影矩阵
        Eigen::Matrix<double,6,6> D_w_u;//U-space投影矩阵
        Eigen::Matrix<double,6,6> D_i;//阻抗方向矩阵
        Eigen::Matrix<double,6,6> D_i_c;//阻抗C-space投影矩阵
        Eigen::Matrix<double,6,6> D_i_u;//阻抗U-space投影矩阵
        Eigen::Matrix<double,6,6> Dd;//阻尼系数d
        Eigen::Matrix<double,6,1> Dd_t_x;//变阻尼系数

        //零空间参数
        Eigen::Matrix<double,7,1> q_current_null;
        Eigen::Matrix<double,7,1> dq_null;
        Eigen::Matrix<double,7,1> q_d_null;
        Eigen::Matrix<double,7,1> tau_null_0;
        Eigen::Matrix<double,7,7> identity_7;
        Eigen::Matrix<double,7,7> null_space_proj;
        Eigen::Matrix<double,7,1> tau_null_projected;
        Eigen::Matrix<double,7,1> tau_cmd_eigen;

        

        //关节相关参数
        std::array<double, 7> q_init;
        std::array<double, 7> dq_init;
        std::array<double, 7> ddq_init;
        
        std::array<double, 7> q_d;
        std::array<double, 7> dq_d;
        std::array<double, 7> ddq_d;

        std::array<double, 7> q_t;
        std::array<double, 7> dq_t;
        std::array<double, 7> ddq_t;

        Eigen::Matrix<double, 7,1> tau_cmd;
        //模型信息
        std::array<double, 49> mass_array;
        std::array<double, 7> coriolis_array;
        std::array<double, 42> jacobian_array;
        std::array<double, 7> gravity_array;
        std::array<double, 42> jacobian_old_array;   
        std::array<double, 7> tau_J_d_array;                           
        std::vector<double> Fd_F_;
        Eigen::Quaterniond orientation_d_;

        double time_count;

        double t_l_f;
        double t_l_i;
        //能量罐阈值参数
        double T_delta_f_h;
        double T_delta_f_s;
        double I_delta_f_h;
        double I_delta_f_s;
        double T_delta_i_h;
        double T_delta_i_s;
        double I_delta_i_h;
        double I_delta_i_s;
        double epsilon_damping;//防止除零
        double T_E_f_u;
        double T_E_i_u;
        double I_E_i_u;
        double I_E_f_u;

        //内部状态变量（用于积分和微分）
        Vector6d F_f_integral_error{};//力控积分项
        Vector6d Fd_prime_last{};     //上一时刻的Fd'(用于求导)
        Vector6d F_ext_last{};        //上一时刻的外部力(用于求导)

        //机械臂结构数据
        Eigen::Matrix<double,6,6>miu;//笛卡尔空间下的科里奥利力
        Eigen::MatrixXd m_mario{};//惯性矩阵
        Eigen::MatrixXd m_mario_pinv{};
        Eigen::MatrixXd c_mario{};//科里奥利力
        Eigen::MatrixXd jacobian_transpose_pinv{};//雅可比矩阵转置伪逆矩阵
        Eigen::MatrixXd jacobian_pinv{};//雅可比矩阵伪逆矩阵
        Eigen::Matrix<double,6,7> J_filter;
        Eigen::Matrix<double,6,7> J_filter_dot;
        Eigen::Matrix<double,6,7> J_new_dot;
        Eigen::MatrixXd Lamada;//惯性矩阵
        bool first_update = true;
        

        //机械臂力矩变化约束
        static constexpr double kDeltaTauMax{1.0};//控制力矩变化的最大值
        Eigen::Matrix<double,7,1> tau_cmd_last;
        Eigen::Matrix<double,7,1> tau_cmd_out;

        //发布机械臂末端力输出值
        ros::Publisher pub_ff_prime_;          // 发布者
        std_msgs::Float64MultiArray msg_ff_prime_; // 消息对象（预分配内存，避免实时循环中new）
        int publish_counter_ = 0;              // 用于降频发送的计数器

        //力控的低通滤波器
        Eigen::Matrix<double,6,1> F_ext_filtered_{};
        double filter_cutoff_freq_ = 20.0;
        bool first_filter_update_ = true;
    };
}