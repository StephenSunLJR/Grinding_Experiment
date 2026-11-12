#include "franka_mujoco_hw.h"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
FrankaMujocoHW::FrankaMujocoHW(): mj_model_(nullptr),mj_data_(nullptr){
    //初始化关节名称，需与URDF/MJCF保持一致
    for (int i=0;i<7;i++){
        joint_name_[i] = "panda_joint"+std::to_string(i+1);
        joint_position_[i] = 0.0;
        joint_velocity_[i] = 0.0;
        joint_effort_[i] = 0.0;
        joint_effort_command_[i] = 0.0;
    }
}

FrankaMujocoHW::~FrankaMujocoHW(){
    if (mj_data_) mj_deleteData(mj_data_);
    if (mj_model_) mj_deleteModel(mj_model_);
}

//这里对NodeHandle进行引用的原因在于其内部有很多状态，如果全量复制很费内存。
bool FrankaMujocoHW::init(ros::NodeHandle& root_nh,ros::NodeHandle& robot_hw_nh){  //&表示引用，意味着函数内部操作的变量就是外部传入的这个变量本身，而不是它的copy

    char error[1000] = "Could not load XML model";
    std::string model_path = "/home/sun/LiuCodeSun/src/franka_ros/franka_mujoco/franka_emika_panda/panda.xml";
    mj_model_ = mj_loadXML(model_path.c_str(),0,error,1000);
    if(! mj_model_){
        ROS_ERROR_STREAM("Load model error"<<error);
        return false;
    }
    mj_data_ = mj_makeData(mj_model_);  //在内存中开辟一块空间来存放关节角度、速度等实时信息。


    // double initial_angles[7] = {0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853};

    // for (int i = 0; i < 7; ++i) {
    //     std::string joint_name = "joint" + std::to_string(i + 1);
    //     // 去 Mujoco 模型里查这个关节的 ID
    //     int jnt_id = mj_name2id(mj_model_, mjOBJ_JOINT, joint_name.c_str());
    //     if (jnt_id >= 0) {
    //         // 获取这个关节在状态数组 qpos 中的内存地址索引
    //         int qpos_adr = mj_model_->jnt_qposadr[jnt_id];
    //         // 强行写入初始角度
    //         mj_data_->qpos[qpos_adr] = initial_angles[i];
    //     } else {
    //         ROS_WARN_STREAM("Do not find joints in Mujoco: " << joint_name);
    //     }
    // }
    // mj_forward(mj_model_, mj_data_);
    
    //注册ROS硬件接口
    for (int i=0;i<7;++i){
        //a.注册状态接口（读取）
        //const std::string& name, const double* pos, const double* vel, const double* eff)
    //: name_(name), pos_(pos), vel_(vel), eff_(eff)
        hardware_interface::JointStateHandle state_handle(
            joint_name_[i],&joint_position_[i],&joint_velocity_[i],&joint_effort_[i]);
        joint_state_interface_.registerHandle(state_handle);//注册句柄.
        //b.注册力矩指令接口（写入）
        hardware_interface::JointHandle effort_handle(
            joint_state_interface_.getHandle(joint_name_[i]),&joint_effort_command_[i]);
        effort_joint_interface_.registerHandle(effort_handle);

        //查找电机在mujoco中的ID
        std::string motor_name = "motor"+std::to_string(i+1);
        motor_ids_[i] = mj_name2id(mj_model_,mjOBJ_ACTUATOR,motor_name.c_str());
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);

    return true;

}

void FrankaMujocoHW::read(const ros::Time &time,const ros::Duration &period){
    //核心：把Mujoco（mj_data_）算出的最新位置/速度，搬运到ROS缓存中
    for(int i=0;i<7;++i){
        joint_position_[i] = mj_data_->qpos[i];//mj_data_是指向mjData结构体的指针，->的含义是去mj_data_指向的内存地址中，找到名为qpos的成员，另外qpos是一个指向数组首地址的指针，[i]是具体的取值动作
        joint_velocity_[i] = mj_data_->qvel[i];
        joint_effort_[i] = mj_data_->qfrc_actuator[i];
    }
}

void FrankaMujocoHW::write(const ros::Time &time,const ros::Duration &period){
    //把ROS controller算出来的力矩，搬运给Mujoco输入端
    for(int i=0;i<7;i++)
    {
        int motor_id = motor_ids_[i];
        if (motor_id>=0){
            mj_data_->ctrl[motor_id] = joint_effort_command_[i] + mj_data_->qfrc_bias[i];//mj_data_->qfrc_bias[i]表示进行了重力补偿
        }
    }
}