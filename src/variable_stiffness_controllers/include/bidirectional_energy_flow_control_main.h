#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <variable_stiffness_controllers/BidirectionalDiagnostics.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_srvs/Trigger.h>

#include "bidirectional_energy_flow_control.h"

namespace variable_stiffness_controllers {

class FrankaModelProvider : public RobotModelProvider {
 public:
  explicit FrankaModelProvider(franka_hw::FrankaModelHandle* model_handle)
      : model_handle_(model_handle) {}

  bool compute(const Vector7d& joint_position,
               const Vector7d& joint_velocity,
               const ModelConfiguration& configuration,
               RobotModelState* model) const override;

 private:
  franka_hw::FrankaModelHandle* model_handle_{nullptr};
};

struct ExternalWrenchSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector6d wrench{Vector6d::Zero()};
  ros::Time stamp{};
  bool valid{false};
};

struct TargetSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ControlTarget target{};
  ros::Time stamp{};
  bool valid{false};
};

/**
 * ROS-control business layer. It snapshots Franka state, external F/T data and
 * the current Cartesian target, invokes the paper controller once, applies
 * hardware safety limits, and writes the resulting torque command.
 */
class BidirectionalEnergyFlowControlMain
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          franka_hw::FrankaStateInterface,
          hardware_interface::EffortJointInterface> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  bool loadParameters(ros::NodeHandle& node_handle);
  bool buildControlInput(const franka::RobotState& robot_state,
                         const Vector6d& external_wrench,
                         ControlInput* input) const;
  Vector7d applyTorqueSafety(const Vector7d& calculated,
                             const Vector7d& previous) const;
  void writeTorqueCommand(const Vector7d& torque);
  void publishDiagnostics(const ros::Time& time, const ros::Duration& period,
                          const franka::RobotState& robot_state,
                          const ExternalWrenchSample* wrench_sample,
                          uint8_t status, bool diagnostics_valid);
  void updateSafeReturn(const franka::RobotState& robot_state,
                        const ros::Time& time,
                        const ros::Duration& period,
                        const Vector7d& previous_torque);
  bool requestSafeStop(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);
  bool safeStopStatus(std_srvs::Trigger::Request& request,
                      std_srvs::Trigger::Response& response);
  void externalWrenchCallback(
      const geometry_msgs::WrenchStampedConstPtr& message);
  void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& message);

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
  std::unique_ptr<FrankaModelProvider> model_provider_;

  BidirectionalEnergyFlowControl algorithm_;
  BidirectionalEnergyFlowParameters algorithm_parameters_{};
  bool algorithm_initialized_{false};
  std::unique_ptr<realtime_tools::RealtimePublisher<BidirectionalDiagnostics>> diagnostics_publisher_;
  uint32_t diagnostics_run_id_{0};
  uint64_t diagnostics_cycle_{0};
  uint64_t diagnostics_drops_{0};
  Vector7d last_commanded_torque_{Vector7d::Zero()};
  Vector7d last_calculated_torque_{Vector7d::Zero()};

  ros::Subscriber external_wrench_subscriber_;
  ros::Subscriber target_pose_subscriber_;
  ros::ServiceServer safe_stop_request_service_;
  ros::ServiceServer safe_stop_status_service_;
  realtime_tools::RealtimeBuffer<ExternalWrenchSample> external_wrench_buffer_;
  realtime_tools::RealtimeBuffer<TargetSample> target_buffer_;

  std::string external_wrench_topic_{"/ft_sensor/wrench"};
  std::string external_wrench_frame_;
  bool external_wrench_in_end_effector_frame_{false};
  std::string target_pose_topic_{"target_pose"};
  std::string target_pose_frame_;
  double external_wrench_timeout_{0.02};
  double external_wrench_sign_{1.0};
  bool require_external_wrench_{true};
  double maximum_torque_step_{1.0};
  Vector7d absolute_torque_limit_{Vector7d::Zero()};

  std::atomic<bool> safe_stop_requested_{false};
  std::atomic<bool> safe_stop_completed_{false};
  std::atomic<bool> controller_running_{false};
  bool safe_return_active_{false};
  ros::Time safe_return_start_time_{};
  double safe_return_settled_duration_{0.0};
  Vector7d initial_joint_position_{Vector7d::Zero()};
  Vector7d safe_return_start_position_{Vector7d::Zero()};
  Vector7d safe_return_position_gain_{Vector7d::Zero()};
  Vector7d safe_return_velocity_gain_{Vector7d::Zero()};
  double safe_return_duration_{5.0};
  double safe_return_position_tolerance_{0.02};
  double safe_return_velocity_tolerance_{0.05};
  double safe_return_settle_time_{0.5};
};

}  // namespace variable_stiffness_controllers
