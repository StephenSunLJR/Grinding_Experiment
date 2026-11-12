#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_mujoco{

    class MujocoImpedanceController:public controller_interface::MultiInterfaceController<
                                            hardware_interface::EffortJointInterface,
                                            hardware_interface::JointStateInterface>{
        


    public:

        bool init(hardware_interface::RobotHW *robot_hw,ros::NodeHandle& node_handle)
        override{
            //获取力矩写入接口
            //这里的robot_hw为指针，所以用->调用里面的方法
            //
            auto *effort_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
            if (!effort_interface){
                ROS_ERROR("Failed to get effort interface!");
                return false;
            }
            //获取要控制的7个关节名称
            std::vector<std::string> joint_names;
            if (!node_handle.getParam("joints",joint_names)||joint_names.size()!=7){
                ROS_ERROR("Invalid joints parameter!");
                return false;
            }
            for(size_t i=0;i<7;++i){
                joint_handles_.push_back(effort_interface->getHandle(joint_names[i]));
            }

            k_gains_ = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
            d_gains_ = {50.0, 50.0, 50.0, 20.0, 20.0, 20.0, 10.0};
            q_d_.resize(7);

            return true;

        }
        void starting(const ros::Time& /*time*/) override{
            // 启动瞬间，记录当前姿态为目标姿态，保持不动
            for (size_t i = 0; i < 7; ++i) {
                q_d_[i] = joint_handles_[i].getPosition();
            }
        }
        void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override {
            for (size_t i = 0; i < 7; ++i) {
                double q = joint_handles_[i].getPosition();
                double dq = joint_handles_[i].getVelocity();
                
                // 核心算法：官方阻抗控制律 (PD)
                double tau = k_gains_[i] * (q_d_[i] - q) - d_gains_[i] * dq;
                
                // MuJoCo 会自动处理重力和科里奥利力，我们直接下发弹簧阻尼力矩！
                joint_handles_[i].setCommand(tau);}
        }
    private:
        std::vector<hardware_interface::JointHandle> joint_handles_;
        std::vector<double> k_gains_;
        std::vector<double> d_gains_;
        std::vector<double> q_d_;
    };

}

PLUGINLIB_EXPORT_CLASS(franka_mujoco::MujocoImpedanceController, controller_interface::ControllerBase)