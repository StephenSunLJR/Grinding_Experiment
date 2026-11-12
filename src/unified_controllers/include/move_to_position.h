#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace unified_controllers {

class MoveToPosition : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> q_start_{{0, 0, 0, 0, 0, 0, 0}};
    double factor_{1.0};


  static constexpr double kDeltaQMotionFinished = 1e-6;
  // std::array<double, 7> q_goal_{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  // std::array<double, 7> q_goal_{{-0.0143379, 0.971493, -0.123065, -1.8724, -0.000344561, 2.8364,0.771321 }};
  std::array<double, 7> q_goal_{{0, 0, 0, 0, 0, 0, 0 }};
  // std::array<double, 7> q_goal_{{-0.169888, 0.840812, 0.122761, -1.59532, -0.299833,  2.40533, 0.870518 }};
   std::array<double, 7> delta_q_{{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> dq_max_sync_{{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> t_1_sync_{{0, 0, 0, 0, 0, 0, 0}};
   std::array<double, 7> t_2_sync_{{0, 0, 0, 0, 0, 0, 0}};
   std::array<double, 7> t_f_sync_{{0, 0, 0, 0, 0, 0, 0}};
   std::array<double, 7> q_1_{{0, 0, 0, 0, 0, 0, 0}};
   std::array<double, 7> t_d_{{0, 0, 0, 0, 0, 0, 0}};
   std::array<int, 7> sign_delta_q{};


    // std::array<double, 7> dq_max_{{2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5}};
    std::array<double, 7> dq_max_{{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}};
    // std::array<double, 7> ddq_max_{{5, 5, 5, 5, 5, 5, 5}};
    std::array<double, 7> ddq_max_{{3, 3, 3, 3, 3, 3, 3}};
};

}  // namespace franka_example_controllers