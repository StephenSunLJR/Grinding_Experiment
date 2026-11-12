#pragma once 
extern "C"{
    #include <mujoco/mujoco.h>
}
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka/robot_state.h>
#include <franka/model.h>

//tips:如果想直接使用官方的控制器，通过init()函数可知，这里编写的代码中必须提供franka_hw::FrankaStateInterface和franka_hw::FrankaModelInterface这两个接口
//前者用来读取底层算法的雅可比矩阵、科里奥利力，后者用来获取动力学模型
class FrankaMujocoHW:public hardware_interface::RobotHW{
public:
    FrankaMujocoHW();
    virtual ~FrankaMujocoHW();  //如果没有virtual，程序只会调用父类的RobotHW的析构函数，而不会理会子类FrankaMujocoHW的析构函数
    //初始化：加载模型、注册ROS接口
    bool init(ros::NodeHandle& root_nh,ros::NodeHandle& robot_hw_nh);
    
    //从MuJoCo读取真实物理状态到 ROS 缓存
    void read(const ros::Time& time,const ros::Duration& period);

    //将ROS Controller计算的力矩下发给Mujoco
    void write(const ros::Time& time,const ros::Duration& period);

    mjModel* mj_model_;
    mjData* mj_data_;
    mjModel* getModel(){return mj_model_;} //
    mjData* getData(){return mj_data_;}
private:
    //ROS标准接口
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    //Franka专属接口：
    franka_hw::FrankaStateInterface franka_state_interface_;
    franka_hw::FrankaModelInterface franka_model_interface_;
    franka::RobotState robot_state{};
    std::string arm_id_{"panda"};

    //内存缓存数组：必须与Mujoco的状态和控制指令对应
    double joint_position_[7];
    double joint_velocity_[7];
    double joint_effort_[7];
    double joint_effort_command_[7]; //controller算出的期望力矩(下发给控制向量)
    std::string joint_name_[7];
    int motor_ids_[7];
};