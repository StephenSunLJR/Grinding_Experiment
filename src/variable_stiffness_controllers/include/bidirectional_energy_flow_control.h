#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace variable_stiffness_controllers {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;
using Matrix76d = Eigen::Matrix<double, 7, 6>;

struct ControlTarget {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Vector6d velocity{Vector6d::Zero()};
  Vector6d acceleration{Vector6d::Zero()};
};

struct ModelConfiguration {
  std::array<double, 9> total_inertia{};
  double total_mass{0.0};
  std::array<double, 3> flange_to_com{};
  std::array<double, 16> flange_to_end_effector{};
  std::array<double, 16> end_effector_to_stiffness{};
};

struct RobotModelState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Matrix7d mass{Matrix7d::Identity()};
  Vector7d coriolis{Vector7d::Zero()};
  Matrix67d jacobian{Matrix67d::Zero()};
  Eigen::Affine3d end_effector_transform{Eigen::Affine3d::Identity()};
  Vector6d end_effector_velocity{Vector6d::Zero()};
};

struct ControlInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector7d joint_position{Vector7d::Zero()};
  Vector7d joint_velocity{Vector7d::Zero()};
  Vector7d previous_desired_torque{Vector7d::Zero()};
  Vector6d external_wrench{Vector6d::Zero()};
  RobotModelState real_model{};
  ModelConfiguration model_configuration{};
};

struct ControlOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector7d total_joint_torque{Vector7d::Zero()};
  bool valid{false};
};

class RobotModelProvider {
 public:
  virtual ~RobotModelProvider() = default;
  virtual bool compute(const Vector7d& joint_position,
                       const Vector7d& joint_velocity,
                       const ModelConfiguration& configuration,
                       RobotModelState* model) const = 0;
};

struct BidirectionalEnergyFlowParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double inertia_scale{1.0};
  Matrix6d task_stiffness{Matrix6d::Zero()};
  Matrix6d task_damping{Matrix6d::Zero()};
  Matrix7d tracking_velocity_gain{Matrix7d::Zero()};
  Matrix7d tracking_position_gain{Matrix7d::Zero()};
  Matrix7d tracking_integral_gain{Matrix7d::Zero()};
  Vector7d tracking_integral_limit{Vector7d::Constant(0.5)};
  Matrix7d nullspace_stiffness{Matrix7d::Zero()};
  Matrix7d nullspace_damping{Matrix7d::Zero()};
  double nullspace_projection_damping{0.01};
  double tank_initial_energy{1.0};
  double tank_lower_bound{0.1};
  double tank_upper_bound{1.0};
  bool filter_alpha{false};
  double alpha_filter_cutoff{20.0};
};

struct ControlDiagnostics {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector7d nominal_joint_position{Vector7d::Zero()};
  Vector7d nominal_joint_velocity{Vector7d::Zero()};
  Vector7d nominal_joint_acceleration{Vector7d::Zero()};
  Vector7d external_joint_torque{Vector7d::Zero()};
  Vector7d nominal_control_torque{Vector7d::Zero()};
  Vector7d compensation_torque{Vector7d::Zero()};
  Vector7d auxiliary_torque{Vector7d::Zero()};
  Vector7d bidirectional_torque{Vector7d::Zero()};
  Vector7d nullspace_torque{Vector7d::Zero()};
  Vector6d real_task_error{Vector6d::Zero()};
  Vector6d nominal_task_error{Vector6d::Zero()};
  double alpha_raw{0.0};
  double alpha{0.0};
  double alpha_dot{0.0};
  double nominal_storage{0.0};
  double estimated_real_storage{0.0};
  double tank_energy{0.0};
  double tank_power{0.0};
  double external_error_power{0.0};
  double compensation_error_power{0.0};
};

/**
 * ROS-independent implementation of Lee et al. (T-RO 2024). The public data
 * flow is measured state + Cartesian target -> total joint torque. Each paper
 * feedback block is implemented by one private method and composed, in paper
 * order, by computeControl().
 */
class BidirectionalEnergyFlowControl {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool configure(const BidirectionalEnergyFlowParameters& parameters,
                 const RobotModelProvider* model_provider);
  bool initialize(const ControlInput& initial_input,
                  const ControlTarget& initial_target);
  ControlOutput computeControl(const ControlInput& input,
                               const ControlTarget& target,
                               double period);
  const ControlDiagnostics& diagnostics() const { return diagnostics_; }

 private:
  bool validateInput(const ControlInput& input,
                     const ControlTarget& target,
                     double period) const;
  bool updateNominalModel(const ControlInput& input);
  void computeTaskSpaceErrors(const ControlInput& input,
                              const ControlTarget& target);
  void computeJointTrackingErrors(const ControlInput& input, double period);
  void computeNominalControlTorque();
  void computeCompensationTorque();
  void computeStorageFunctions(const ControlInput& input);
  void computeEnergyFlowIndicators();
  void updateModulationAndTank(const ControlInput& input, double period);
  void computeAuxiliaryTorque(const ControlInput& input);
  void computeBidirectionalTorque();
  bool computeNullspaceTorque(const ControlInput& input);
  bool computeNominalAcceleration(const ControlInput& input, double period);
  void integrateNominalRobot(double period);
  ControlOutput computeTotalMotorTorque() const;

  static Vector6d computePoseError(const ControlTarget& target,
                                   const Eigen::Affine3d& actual_transform);
  static double clamp(double value, double lower, double upper);

  BidirectionalEnergyFlowParameters parameters_{};
  const RobotModelProvider* model_provider_{nullptr};
  bool configured_{false};
  bool initialized_{false};

  Vector7d nominal_joint_position_{Vector7d::Zero()};
  Vector7d nominal_joint_velocity_{Vector7d::Zero()};
  Vector7d nominal_joint_acceleration_{Vector7d::Zero()};
  Vector7d nullspace_reference_{Vector7d::Zero()};
  RobotModelState nominal_model_{};
  Vector6d real_task_error_{Vector6d::Zero()};
  Vector6d nominal_task_error_{Vector6d::Zero()};
  Vector7d tracking_error_{Vector7d::Zero()};
  Vector7d tracking_velocity_error_{Vector7d::Zero()};
  Vector7d tracking_error_integral_{Vector7d::Zero()};
  Vector7d external_joint_torque_{Vector7d::Zero()};
  Vector7d nominal_control_torque_{Vector7d::Zero()};
  Vector7d compensation_torque_{Vector7d::Zero()};
  Vector7d auxiliary_torque_{Vector7d::Zero()};
  Vector7d bidirectional_torque_{Vector7d::Zero()};
  Vector7d nullspace_torque_{Vector7d::Zero()};

  double alpha_raw_{0.0};
  double alpha_{0.0};
  double alpha_previous_{0.0};
  double alpha_dot_{0.0};
  double nominal_storage_{0.0};
  double estimated_real_storage_{0.0};
  double tank_state_{0.0};
  double tank_energy_{0.0};
  double tank_power_{0.0};
  double external_error_power_{0.0};
  double compensation_error_power_{0.0};
  double unopposed_generation_power_{0.0};
  ControlDiagnostics diagnostics_{};
};

}  // namespace variable_stiffness_controllers
