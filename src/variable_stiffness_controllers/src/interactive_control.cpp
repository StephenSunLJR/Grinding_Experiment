#include "../include/interactive_control.h"
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
#include <std_msgs/String.h>
#include <hardware_interface/robot_hw.h>

// 实现思路：
/*
    1. 获取模型接口和状态接口
    2. 变量初始化
    3. 全局变量的定义放在头文件中
*/
/*
    现象：对机械臂进行移动之后，机械臂落下之后就很难抬起来：
    原因：机械臂不能落下，力控中的积分项会将这些误差一直累加，到后面越来越大，所以抬不起来
    现象追问：为什么误差很大，但是机械臂末端不会下落？是不是和d_f参数有关，进而和T_E_f的补充有关
*/

namespace interactive_force_impedence_control
{
    double interactive_control::calculate_damping(double S_E_f, double S_E_f_l,
                                                  double S_delta_h, double S_delta_s, double p, double epsilon)
    {
        if (S_E_f > S_E_f_l + S_delta_h)
        {
            return 1.0;
        }
        else if (S_E_f < S_E_f_l + S_delta_s)
        {
            return 1.0 / epsilon;
        }
        else
        {
            // 对应 Eq. (21)
            double A = (S_E_f - S_E_f_l - S_delta_s) / (S_delta_h - S_delta_s);
            double cos_term = std::cos((1.0 - std::pow(A, p)) * M_PI / 2.0);
            // 对应 Eq. (20)
            return 1.0 / (cos_term + epsilon);
        }
    }

    bool interactive_control::init(hardware_interface::RobotHW *robot_hw,
                                   ros::NodeHandle &node_handle)
    {
        std::string arm_id;
        std::vector<std::string> joint_names;
        pub_ff_prime_ = node_handle.advertise<std_msgs::Float64MultiArray>("ff_prime_debug", 100);
        msg_ff_prime_.data.resize(6);
        if (!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter arm_id");
            return false;
        }

        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
        {
            ROS_ERROR("interactive_force_impedence_control: Invalid or no joint_names parameters provided, aborting controller init");
            return false;
        }
        // 获取模型接口和状态接口
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "interactive_force_impedence_control: Error getting model interface from hardware");
            return false;
        }

        // 获取机器人模型句柄
        try
        {
            // 这里的model_handle_是interactive_control类的一个私有成员变量，但是因为init函数也是该类的方法，所以可以对model_handle_进行赋值操作
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                model_interface->getHandle(arm_id + "_model")); // 这里的<>后括号中的内容是构造函数参数
            // getHandle("panda_robot") 会在这个管理器中查找名为 "panda_robot" 的硬件资源，并返回一个 franka_hw::FrankaStateHandle 对象。
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM(
                "interactive_force_impedence_control: Exception getting model handle from interface: "
                << ex.what());
            return false;
        }
        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr)
        {
            ROS_ERROR_STREAM("interactive_force_impedence_control: Error getting state interface from hardware");
            return false;
        }

        // 获取机器人状态句柄
        try
        {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM("interactive_force_impedence_control: Exception getting state handle from interface: " << ex.what());
            return false;
        }
        // 获取关节接口
        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM("interactive_force_impedence_control: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i)
        {
            try
            {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
                // 这里的getHandle通过关节名称joint_names[i]在ROS硬件层中查找并返回一个与该关节绑定的“句柄”，
                // 该句柄包含了该关节的所有信息，包括名称、状态、目标值等等
                // push_back将元素加入到vector的末尾
            }
            catch (const hardware_interface::HardwareInterfaceException &ex)
            {
                ROS_ERROR_STREAM("interactive_force_impedence_control: Exception getting joint handles: " << ex.what());
                return false;
            }
        }
        // 参数接收判断

        std::vector<double> Kp_;
        std::vector<double> Kd_;
        std::vector<double> Ki_;

        if (!node_handle.getParam("Kp", Kp_))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter Kp");
            return false;
        }
        if (!node_handle.getParam("Ki", Ki_))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter Kp");
            return false;
        }
        if (!node_handle.getParam("Kd", Kd_))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter Kp");
            return false;
        }
        // 将容器中的数据映射到矩阵中
        Kp = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Kp_.data());
        Ki = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Ki_.data());
        Kd = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Kd_.data());

        if (!node_handle.getParam("p_v_f", P_Vf))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter P_Vf");
            return false;
        }
        if (!node_handle.getParam("p_v_i", P_Vi))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter P_Vi");
            return false;
        }
        if (!node_handle.getParam("T_E_f", T_E_f))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter T_E_f");
            return false;
        }
        if (!node_handle.getParam("I_E_f", I_E_f))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter I_E_f");
            return false;
        }
        if (!node_handle.getParam("T_E_i", T_E_i))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter T_E_i");
            return false;
        }
        if (!node_handle.getParam("I_E_i", I_E_i))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter I_E_i");
            return false;
        }
        if (!node_handle.getParam("T_E_f_l", T_E_f_l))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter T_E_f_l");
            return false;
        }
        if (!node_handle.getParam("I_E_f_l", I_E_f_l))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter I_E_f_l");
            return false;
        }
        if (!node_handle.getParam("T_E_i_l", T_E_i_l))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter T_E_i_l");
            return false;
        }
        if (!node_handle.getParam("I_E_i_l", I_E_i_l))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter I_E_i_l");
            return false;
        }
        node_handle.getParam("epsilon_damping", epsilon_damping);
        node_handle.getParam("T_delta_f_h", T_delta_f_h);
        node_handle.getParam("T_delta_i_h", T_delta_i_h);
        node_handle.getParam("T_delta_f_s", T_delta_f_s);
        node_handle.getParam("T_delta_i_s", T_delta_i_s);
        node_handle.getParam("I_delta_f_h", I_delta_f_h);
        node_handle.getParam("I_delta_i_h", I_delta_i_h);
        node_handle.getParam("I_delta_f_s", I_delta_f_s);
        node_handle.getParam("I_delta_i_s", I_delta_i_s);
        node_handle.getParam("T_E_f_u", T_E_f_u);
        node_handle.getParam("T_E_i_u", T_E_i_u);
        node_handle.getParam("I_E_f_u", I_E_f_u);
        node_handle.getParam("I_E_i_u", I_E_i_u);

        std::cout << "epsilon_damping:" << epsilon_damping << std::endl;
        // 能量罐状态
        z_f = std::sqrt(2 * T_E_f);
        z_i = std::sqrt(2 * T_E_i);
        epsilon_damping = 0.0001;

        if (!node_handle.getParam("Fd_f", Fd_F_))
        {
            ROS_ERROR("interactive_force_impedence_control: Could not read parameter Fd_F");
            return false;
        }

        return true;
    }

    void interactive_control::starting(const ros::Time & /*time*/)
    {
        ros::NodeHandle nhpub;
        pub = nhpub.advertise<variable_stiffness_controllers::compliance_robot>("interactive_robot", 10);
        franka::RobotState robot_state = state_handle_->getRobotState(); // 获取机器人状态
        // 按照需求选择是否要进行订阅
        q_init = robot_state.q;
        dq_init = robot_state.dq;
        ddq_init = {0, 0, 0, 0, 0, 0, 0};

        q_d = q_init;
        dq_d = dq_init;
        ddq_d = ddq_init;

        // 设置初始值
        t = ros::Duration(0.0);

        // 只有基于Eigen库的变量才能使用.setZero函数
        // Fd_F.setZero();
        // 变量初始化
        // 机械臂状态变量初始化
        position_d.setZero();
        position_d_dot.setZero();
        position_d_ddot.setZero();
        pose_tilde.setZero();
        pose_tilde_dot.setZero();
        F_ext.setZero();

        tau_cmd_last.setZero();
        tau_cmd_out.setZero();

        // 投影矩阵初始化
        Fd_bin << 0, 0, 1, 0, 0, 0; // 假设力控Z轴与世界坐标系的Z轴平行
        D_i.setZero();
        D_i_c.setZero();
        D_i_u.setZero();
        // Di与Dw相同？这里存疑
        D_w.setZero();
        D_w_c.setZero();
        D_w_u.setZero();
        K_s.setZero();
        R6_W_F_i.setZero();
        R6_W_F_w.setZero();
        F_ext.setZero();
        Dd.setZero(); // 阻尼系数
        m_mario_pinv.setZero();
        position_init.setZero();
        Dd.diagonal() << 16.0, 16.0, 16.0, 12.0, 12.0, 12.0;
        K_s.diagonal() << 200.0, 200.0, 200.0, 160.0, 160.0, 160.0;

        // 零空间参数初始化
        q_current_null.setZero();
        dq_null.setZero();
        q_d_null.setZero();
        tau_null_0.setZero();
        identity_7.setZero();
        null_space_proj.setZero();
        tau_null_projected.setZero();
        tau_cmd_eigen.setZero();

        // 初始化末端位姿
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        position_init = initial_transform.translation();
        position_d = initial_transform.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
        jacobian_old_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        tau_cmd.setZero();
        first_update = true;
    }
    void interactive_control::update(const ros::Time & /*time*/, const ros::Duration &period)
    {
        /*
            update函数的作用：
                1.获取机器人状态
                2.进行控制律输出运算
                3.输出力矩命令
        */
        time_count += period.toSec(); // 计算特定的运行时间，并在需要时可进行重置，可用于频率限制
        t += period;                  // 计算运行总时间
        franka::RobotState robot_state = state_handle_->getRobotState();
        q_t = robot_state.q; // 用代码代替注释
        dq_t = robot_state.dq;
        tau_J_d_array = robot_state.tau_J_d;
        /*
        position_d[0] = position_init[0] + radius_*cos(factor_circle*t.toSec());
        position_d[1] = position_init[1] + radius_*sin(factor_circle*t.toSec());
        */
        // 获取模型信息
        mass_array = model_handle_->getMass();
        coriolis_array = model_handle_->getCoriolis();
        gravity_array = model_handle_->getGravity();
        // Eigen映射
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_t.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_t.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(tau_J_d_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
        // 机械臂运动学信息
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d x(transform.translation());
        Eigen::Matrix3d transformation(transform.rotation());
        Eigen::Quaterniond orientation(transform.rotation());

        double k_null = 1.0;
        double d_null = 0.5;

        double dx_d = 0.0;
        double dy_d = 0.0; // factor_circle*radius_*cos(factor_circle*t.toSec());
        double dz_d = 0.0; //-factor_circle*radius_*cos(factor_circle*t.toSec());
        position_d_dot << dx_d, dy_d, dz_d, 0.0, 0.0, 0.0;

        double ddx_d = 0.0; //-factor_circle*factor_circle*radius_*sin(factor_circle*t.toSec());
        double ddy_d = 0.0; // factor_circle*factor_circle*radius_*sin(factor_circle*t.toSec());
        double ddz_d = 0.0;

        position_d_ddot << ddx_d, ddy_d, ddz_d, 0.0, 0.0, 0.0;

        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_old_array.data());

        // 获取外力
        Eigen::Map<Vector6d> Fd_F(Fd_F_.data());                                             // 这里的Map函数将Fd_F_中的数据映射到Vector6d中，同时定义了变量Fd_F,Fd_F_与Fd_F共用一块内存空间
        Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext_raw(robot_state.O_F_ext_hat_K.data()); // franka机械臂没有外力传感器，这里为估算的外力，机械臂自身产生的力为正，环境作用力为负。
        double dt = period.toSec();
        dt = 0.001;

        // 低通滤波
        double rc = 1.0 / (2.0 * M_PI * filter_cutoff_freq_);
        double alpha = dt / (rc + dt);
        // 如果是第一次运行，直接将滤波值初始化为当前测量值，防止突变
        if (first_filter_update_)
        {
            F_ext_filtered_ = F_ext_raw;
            first_filter_update_ = false;
        }
        else
        {
            // 一阶低通滤波公式: y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
            F_ext_filtered_ = alpha * F_ext_raw + (1.0 - alpha) * F_ext_filtered_;
        }
        Vector6d F_ext = -F_ext_filtered_;
        //ROS_ERROR_STREAM_THROTTLE(0.5, "F_ext: " << F_ext);

        Vector6d Fd_bin_impedence = Vector6d::Ones() - Fd_bin;
        calculate_universal_impedence_matrices(transformation, Fd_bin_impedence, D_i, D_i_c, D_i_u, R6_W_F_i);
        calculate_universal_impedence_matrices(transformation, Fd_bin, D_w, D_w_c, D_w_u, R6_W_F_w);

        // ROS_ERROR_STREAM_THROTTLE(0.5, "transformation: " << transformation);
        // ROS_ERROR_STREAM_THROTTLE(0.5, "R6_W_F_w: " << R6_W_F_w);
        // ROS_ERROR_STREAM_THROTTLE(0.5, "D_w_c: " << D_w_c);
        // ROS_ERROR_STREAM_THROTTLE(0.5, "D_w_u*D_w_c: " << D_w_u*D_w_c);

        Vector6d x_dot = jacobian * dq;
        double r1 = 0.01;

        //----补充：计算系统的结构参数------d
        // 对雅可比矩阵进行滤波
        if (first_update)
        {
            J_filter = jacobian;
            J_new_dot.setZero();
            J_filter_dot.setZero();
        }
        else
        {
            J_new_dot = (jacobian - jacobian_old) / dt;
            J_filter_dot = (jacobian - J_filter) / r1;
            J_filter = J_filter_dot * dt + J_filter;
        }

        jacobian_old_array = jacobian_array;
        first_update = false;
        franka_interactive_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
        franka_interactive_controllers::pseudoInverse(jacobian, jacobian_pinv);

        m_mario = MassMatrix(q);
        c_mario = CoriolisMatrix(q, dq);
        miu = jacobian_transpose_pinv * (c_mario - m_mario * jacobian_pinv * J_filter_dot) * jacobian_pinv;
        franka_interactive_controllers::pseudoInverse(m_mario, m_mario_pinv);
        /*
        if(time_count<1.0)
        {
            miu.setZero();
            Lamada.setZero();
        }
        else
        {
            franka_interactive_controllers::pseudoInverse(jacobian*m_mario_pinv*jacobian_pinv,Lamada);
            miu=jacobian_transpose_pinv*(c_mario-m_mario*jacobian_pinv*J_filter_dot)*jacobian_pinv;
        }
        */

        franka_interactive_controllers::pseudoInverse(jacobian * m_mario_pinv * jacobian_pinv, Lamada);
        // --- 2. 计算交互功率 (P_c, P_u) ---
        // 对应 Eq. (53)

        Pc = x_dot.transpose() * D_w_c * F_ext;

        // F_ext.setZero();
        //  对应 Eq. (53)
        Pu = x_dot.transpose() * D_w_u * F_ext;

        double k_p_P_c = x_dot.transpose() * (Kp * D_w_c * F_ext);
        double k_p_prime_P_u = x_dot.transpose() * (Kp * D_w_u * F_ext);

        // --- 3. 更新力控能量罐 (Force Tank) ---

        // 3a. 更新交互腔室 (I_E_f)
        // 对应 Eq. (19)
        lamada_Pc = (Pc > 0.0) ? 1.0 : 0.0;
        double P_Vf_current = (Pc < P_Vf) ? (I_E_f_u / t_l_f) : P_Vf;
       
        // 对应 Eq. (17)
        I_E_f += (P_Vf_current - lamada_Pc * k_p_P_c) * dt;
        I_E_f = std::max(0.0, std::min(I_E_f, I_E_f_u));

        // 3b. 计算力控阻尼 (d_f_T, d_f_I)
        // 对应 Eq. (21)
        double p_f = (Pc > 0.1) ? 1.0 : 10.0;

        // 对应 Eq. (20)
        d_f_T = calculate_damping(T_E_f, T_E_f_l, T_delta_f_h, T_delta_f_s, p_f, epsilon_damping);
        // 对应 Eq. (20)
        d_f_I = calculate_damping(I_E_f, I_E_f_l, I_delta_f_h, I_delta_f_s, p_f, epsilon_damping);
        double d_f = d_f_T * d_f_I;

        // --- 4. 计算原始力控输出 (F_f) ---

        Vector6d Fd = Fd_F;
        // 对应 Eq. (4)  ---------------------注意这个公式的正确性------------------------------//
        Vector6d Fd_front;
        Fd_front<<0,0,0,0,0,0;
        Fd_prime = (Fd_bin.array() - 1.0).matrix().cwiseProduct(F_ext) + Fd +Fd_front;
        // ROS_ERROR_STREAM_THROTTLE(0.5, "Fd_prime: " << Fd_prime);
        //  对应 Eq. (5)
        F_f_integral_error += (Fd_prime + F_ext) * dt;
        Vector6d Fd_prime_dot = (Fd_prime - Fd_prime_last) / dt;
        Vector6d F_ext_dot = (F_ext - F_ext_last) / dt;
        //ROS_ERROR_STREAM_THROTTLE(0.5,"Fd_prime_dot + F_ext_dot: "<<Fd_prime_dot + F_ext_dot);
        //ROS_ERROR_STREAM_THROTTLE(0.5, "F_f_integral_error: " << F_f_integral_error<<" | Fd_prime_dot: "<<Fd_prime_dot<<" | F_ext_dot: "<<F_ext_dot<<endl);
        Kd.setZero();
        //Ki.setZero();

        Ff_i_d = Ki * F_f_integral_error + Kd * (Fd_prime_dot + F_ext_dot) + Fd + Fd_front; // 这里的Ki和Kd的计算方法未与论文中统一
        // ROS_ERROR_STREAM_THROTTLE(0.5, "Ff_i_d: " << Ff_i_d);

        // 对应 Eq. (3)
        Vector6d F_f = Kp * (F_ext + Fd_prime) + Ff_i_d;

        // --- 5. 计算修正后的力 (Ff_prime) ---
        // 对应 Eq. (22)
        Ff_prime = F_f / d_f;
        // ROS_ERROR_STREAM_THROTTLE(0.5, "d_f: " << d_f);

        // --- 6. 更新阻抗能量罐 (Impedance Tank) ---

        // 6a. 更新交互腔室 (I_E_i)
        double lambda_Pu = (Pu > 0) ? 1.0 : 0.0;
        double P_Vi_current = (Pu < 0) ? (I_E_i_u / t_l_i) : P_Vi;
        I_E_i += (P_Vi_current - lambda_Pu * k_p_prime_P_u) * dt;
        I_E_i = std::max(0.0, std::min(I_E_i, I_E_i_u));

        // 6b. 计算阻抗阻尼 (d_i_T, d_i_I)
        double p_i = (Pu > 0) ? 1.0 : 10.0;
        d_i_T = calculate_damping(T_E_i, T_E_i_l,  T_delta_i_h, T_delta_i_s, p_i, epsilon_damping);
        d_i_I = calculate_damping(I_E_i, I_E_i_l,  I_delta_i_h, I_delta_i_s, p_i, epsilon_damping);
        double d_i = d_i_T * d_i_I;

        // --- 7. 计算修正后的期望速度 (xd_dot_prime) ---
        // 对应 Eq. (24)
        xd_dot_prime = position_d_dot / d_i;

        // --- 8. 更新总能量罐 (T_E_f, T_E_i) ---

        // 8a. 更新 T_E_i (阻抗任务罐)
        Vector6d x_tilde_dot_prime = x_dot - xd_dot_prime;
        // ROS_ERROR_STREAM_THROTTLE(0.5, "x_tilde_dot_prime: "<< x_tilde_dot_prime);
        //  对应 Eq. (23)
        P_i = xd_dot_prime.dot(Ff_prime + F_ext) + x_tilde_dot_prime.dot((Dd * D_i_c) * x_tilde_dot_prime); // 如果是xd_dot.transport()*(Ff_prime + F_ext),最后得到的结果类型为1x1的matrix而非double
        // 对应 Eq. (23)
        T_E_i += P_i * dt;
        T_E_i = std::max(0.0, std::min(T_E_i, T_E_i_u)); // 保证能量罐中总能量在允许范围内
        z_i = std::sqrt(2 * T_E_i);

        // 8b. 更新 T_E_f (力控任务罐)
        // 对应 Eq. (18)
        lambda_c = (I_E_f < I_E_f_u - 0.01) ? 1.0 : 0.0;
        Vector6d F_f_r = Kp * D_w_c * Fd_prime + Ff_i_d;
        // 对应 Eq. (17)
        P_f = -x_dot.transpose().dot(F_f_r) + lambda_c * x_dot.dot((Dd * D_i_u) * x_dot) - k_p_P_c;
        // E_q. (16)
        T_E_f += P_f * dt / d_f;
        T_E_f = std::max(0.0, std::min(T_E_f, T_E_f_u));

        //  对应 Eq. (156) (注：原文为 1/2 z_f^2，这里是 z_f 的更新)
        z_f = std::sqrt(2 * T_E_f);

        // --- 9. 计算最终控制力矩 (Tau_cmd) ---

        // 9a. 计算位置和姿态误差 (x_tilde)
        Eigen::Vector3d x_tilde_pos = x - position_d;
        Eigen::Quaterniond o_tilde = orientation.conjugate() * orientation_d_;
        // 防止四元数双倍覆盖问题 (确保走最短路径)
        if (o_tilde.w() < 0.0)
        {
            o_tilde.coeffs() = -o_tilde.coeffs();
        }
        Eigen::Vector3d x_tilde_rot;
        x_tilde_rot << o_tilde.x(), o_tilde.y(), o_tilde.z();
        x_tilde_rot = orientation.matrix() * x_tilde_rot;

        pose_tilde.head(3) << x_tilde_pos;
        pose_tilde.tail(3) << -x_tilde_rot; // 这里这个负号一定不要忘！！！！
        // ROS_ERROR_STREAM_THROTTLE(0.5, "pose_tilde_x: "<< pose_tilde[0]<<"pose_tilde_y: "<< pose_tilde[1]);

        // 9b. 计算阻抗力 (F_imp)
        // 对应 Eq. (8)
        // 变阻尼
        position_dot = jacobian * dq;
        Dd_t_x = lambda_c * Dd * D_i_u * position_dot + Dd * D_i_c * x_tilde_dot_prime;
        // Dd_t_x = Dd * x_tilde_dot_prime;
        // Vector6d damping_energy_break = lambda_c * 2.0 * Dd * position_dot; // 系数 2.0 可调

        /*
        Vector6d damping_c = Dd * D_i_c * x_tilde_dot_prime;
        Vector6d damping_u_base = Dd * D_i_u * x_tilde_dot_prime;
        Dd_t_x = damping_c + damping_u_base ;//+ damping_energy_break;
        */

        // pose_tilde.setZero();
        // x_tilde_dot_prime.setZero();

        // 零空间控制
        q_d_null = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_init.data()); // 目标设为初始位姿

        q_current_null = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_t.data());
        dq_null = Eigen::Map<Eigen::Matrix<double, 7, 1>>(dq_t.data());
        q_d_null = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_init.data()); // 目标设为初始位姿
        tau_null_0 = -k_null * (q_current_null - q_d_null) - d_null * dq_null;
        identity_7 = Eigen::Matrix<double, 7, 7>::Identity();
        null_space_proj = identity_7 - jacobian.transpose() * jacobian_transpose_pinv;
        // tau_null_projected = null_space_proj * tau_null_0;

        tau_null_projected << (Eigen::MatrixXd::Identity(7, 7) -
                               jacobian.transpose() * jacobian_transpose_pinv) *
                                  (k_null * (q_d_null - q) -
                                   (2.0 * sqrt(k_null)) * dq);

        Vector6d F_imp = -K_s * pose_tilde - Dd_t_x + miu * xd_dot_prime + Lamada * position_d_ddot;
        // Vector6d F_imp = -K_s * pose_tilde - Dd * x_tilde_dot_prime + miu * xd_dot_prime + Lamada * position_d_ddot;
        // F_imp.setZero();
        //  9c. 笛卡尔总控制 wrench (F_cartesian)
        double F_z = (D_w_c * Ff_prime)[2];
        Vector6d Ff_out ;
        Ff_out<<0.0,0.0,F_z,0.0,0.0,0.0;
         Vector6d F_cartesian =D_i_c * F_imp + Ff_out;
         ROS_ERROR_STREAM_THROTTLE(0.5,"Ff_out: "<<Ff_out);
        // Vector6d F_cartesian = F_imp ;
        //Vector6d F_cartesian = Ff_prime;
        ROS_ERROR_STREAM_THROTTLE(0.5, "D_i_c * F_imp: "<<D_i_c * F_imp);
        ROS_ERROR_STREAM_THROTTLE(0.5, "T_E_f:" << T_E_f << "|I_E_f" << I_E_f << "|Pc:" << Pc << "|d_f: " << d_f<<"|d_f_T: "<<d_f_T<<"|d_f_I: "<<d_f_I<<"P_f: "<<P_f<<"|P_Vf: "<<P_Vf);
        // Vector6d F_cartesian = F_imp;
        tau_cmd_eigen = jacobian.transpose() * F_cartesian + tau_null_projected; //+ coriolis;//+ gravity;

        // 9d. 计算最终关节力矩
        // 对应 Eq. (60), (81), (84) (注：这些是第II节的公式)
        // Eigen::Vector7d tau_cmd_eigen;
        // tau_cmd_eigen = jacobian.transpose() * F_cartesian;// + gravity ;
        //-------------------------------------------------------------------------------------------------
        // tau_cmd_eigen = gravity;

        // --- 10. 发送力矩命令 ---
        tau_cmd_out = interactive_control::saturateTorqueRate(tau_cmd_eigen, tau_cmd_last);
        // ROS_ERROR_STREAM_THROTTLE(0.5, "tau_cmd_out: "<<tau_cmd_out);
        for (size_t i = 0; i < 7; ++i)
        {
            tau_cmd[i] = tau_cmd_out[i];
            joint_handles_[i].setCommand(tau_cmd[i]);
        }
        for (size_t i = 0; i < 7; ++i)
        {
            tau_cmd_last[i] = tau_cmd[i];
        }

        // --- 11. 更新历史变量 ---
        Fd_prime_last = Fd_prime;
        ROS_ERROR_STREAM_THROTTLE(0.5, "F_ext: "<<F_ext);
        F_ext_last = F_ext;

        if (publish_counter_++ > 10)
        {
            publish_counter_ = 0;

            // 将 Eigen 数据拷贝到 ROS 消息中
            for (size_t i = 0; i < 6; ++i)
            {
                msg_ff_prime_.data[i] = Ff_prime[i];
            }

            // 发布消息
            pub_ff_prime_.publish(msg_ff_prime_);
        }
    }

    // 添加力矩限制
    Eigen::Matrix<double, 7, 1> interactive_control::saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d)
    {
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++)
        {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax); // 控制力矩变化率
            // 以下的力矩限制可以通过自己的需要更改，详见fci
            if (i == 0 || i == 1 || i == 2 || i == 3)
            {
                if (tau_d_saturated[i] > 30)
                {
                    tau_d_saturated[i] = 30;
                }
                if (tau_d_saturated[i] < -30)
                {
                    tau_d_saturated[i] = -30;
                }
            }
            else
            {
                if (tau_d_saturated[i] > 8)
                {
                    tau_d_saturated[i] = 8;
                }
                if (tau_d_saturated[i] < -8)
                {
                    tau_d_saturated[i] = -8;
                }
            }
        }
        return tau_d_saturated;
    }

}

PLUGINLIB_EXPORT_CLASS(interactive_force_impedence_control::interactive_control,
                       controller_interface::ControllerBase);