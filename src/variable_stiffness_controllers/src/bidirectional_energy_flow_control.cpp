#include "../include/bidirectional_energy_flow_control.h"

#include <algorithm>
#include <cmath>

namespace variable_stiffness_controllers {
namespace {
constexpr double kNumericalEpsilon = 1.0e-9;
constexpr double kMaximumPeriod = 0.1;
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
}  // namespace

bool BidirectionalEnergyFlowControl::configure(
    const BidirectionalEnergyFlowParameters& parameters,
    const RobotModelProvider* model_provider) {
  if (model_provider == nullptr || parameters.inertia_scale <= 0.0 ||
      parameters.tank_initial_energy < 0.0 ||
      parameters.tank_lower_bound <= 0.0 ||
      parameters.tank_upper_bound <= parameters.tank_lower_bound ||
      parameters.tank_initial_energy > parameters.tank_upper_bound ||
      parameters.alpha_filter_cutoff <= 0.0 ||
      !parameters.task_stiffness.allFinite() ||
      !parameters.task_damping.allFinite() ||
      !parameters.tracking_velocity_gain.allFinite() ||
      !parameters.tracking_position_gain.allFinite() ||
      !parameters.tracking_integral_gain.allFinite() ||
      !parameters.tracking_integral_limit.allFinite() ||
      !parameters.nullspace_stiffness.allFinite() ||
      !parameters.nullspace_damping.allFinite() ||
      parameters.nullspace_projection_damping <= 0.0 ||
      (parameters.nullspace_stiffness.diagonal().array() < 0.0).any() ||
      (parameters.nullspace_damping.diagonal().array() < 0.0).any()) {
    return false;
  }
  parameters_ = parameters;
  model_provider_ = model_provider;
  configured_ = true;
  initialized_ = false;
  return true;
}

bool BidirectionalEnergyFlowControl::initialize(
    const ControlInput& initial_input, const ControlTarget& initial_target) {
  if (!configured_ || !validateInput(initial_input, initial_target, 1.0e-3)) {
    return false;
  }
  nominal_joint_position_ = initial_input.joint_position;
  nominal_joint_velocity_ = initial_input.joint_velocity;
  nominal_joint_acceleration_.setZero();
  nullspace_reference_ = initial_input.joint_position;
  nullspace_torque_.setZero();
  tracking_error_integral_.setZero();
  alpha_raw_ = alpha_ = alpha_previous_ = alpha_dot_ = 0.0;
  tank_energy_ = parameters_.tank_initial_energy;
  tank_state_ = std::sqrt(2.0 * tank_energy_);
  tank_power_ = 0.0;
  initialized_ = updateNominalModel(initial_input);
  return initialized_;
}

ControlOutput BidirectionalEnergyFlowControl::computeControl(
    const ControlInput& input, const ControlTarget& target, double period) {
  ControlOutput invalid;
  if (!initialized_ || !validateInput(input, target, period) ||
      !updateNominalModel(input)) {
    return invalid;
  }

  // Complete paper data flow. Hardware safety limiting is intentionally left
  // to the ROS/Franka business layer.
  external_joint_torque_.noalias() =
      input.real_model.jacobian.transpose() * input.external_wrench;
  computeTaskSpaceErrors(input, target);
  computeJointTrackingErrors(input, period);
  computeNominalControlTorque();
  computeCompensationTorque();
  computeStorageFunctions(input);
  computeEnergyFlowIndicators();
  updateModulationAndTank(input, period);
  computeAuxiliaryTorque(input);
  computeBidirectionalTorque();
  if (!computeNullspaceTorque(input)) {
    return invalid;
  }
  if (!computeNominalAcceleration(input, period)) {
    return invalid;
  }
  ControlOutput output = computeTotalMotorTorque();
  if (!output.valid) {
    return output;
  }
  integrateNominalRobot(period);

  diagnostics_.nominal_joint_position = nominal_joint_position_;
  diagnostics_.nominal_joint_velocity = nominal_joint_velocity_;
  diagnostics_.nominal_joint_acceleration = nominal_joint_acceleration_;
  diagnostics_.external_joint_torque = external_joint_torque_;
  diagnostics_.nominal_control_torque = nominal_control_torque_;
  diagnostics_.compensation_torque = compensation_torque_;
  diagnostics_.auxiliary_torque = auxiliary_torque_;
  diagnostics_.bidirectional_torque = bidirectional_torque_;
  diagnostics_.nullspace_torque = nullspace_torque_;
  diagnostics_.real_task_error = real_task_error_;
  diagnostics_.nominal_task_error = nominal_task_error_;
  diagnostics_.alpha_raw = alpha_raw_;
  diagnostics_.alpha = alpha_;
  diagnostics_.alpha_dot = alpha_dot_;
  diagnostics_.nominal_storage = nominal_storage_;
  diagnostics_.estimated_real_storage = estimated_real_storage_;
  diagnostics_.tank_energy = tank_energy_;
  diagnostics_.tank_power = tank_power_;
  diagnostics_.external_error_power = external_error_power_;
  diagnostics_.compensation_error_power = compensation_error_power_;
  return output;
}

bool BidirectionalEnergyFlowControl::validateInput(
    const ControlInput& input, const ControlTarget& target,
    double period) const {
  return period > 0.0 && period <= kMaximumPeriod &&
         input.joint_position.allFinite() && input.joint_velocity.allFinite() &&
         input.previous_desired_torque.allFinite() &&
         input.external_wrench.allFinite() && input.real_model.mass.allFinite() &&
         input.real_model.coriolis.allFinite() &&
         input.real_model.jacobian.allFinite() &&
         input.real_model.end_effector_transform.matrix().allFinite() &&
         input.real_model.end_effector_velocity.allFinite() &&
         target.position.allFinite() && target.orientation.coeffs().allFinite() &&
         target.orientation.norm() > kNumericalEpsilon &&
         target.velocity.allFinite() && target.acceleration.allFinite();
}

bool BidirectionalEnergyFlowControl::updateNominalModel(
    const ControlInput& input) {
  if (!model_provider_->compute(nominal_joint_position_,
                                nominal_joint_velocity_,
                                input.model_configuration, &nominal_model_)) {
    return false;
  }
  nominal_model_.mass *= parameters_.inertia_scale;
  nominal_model_.coriolis *= parameters_.inertia_scale;
  return nominal_model_.mass.allFinite() && nominal_model_.coriolis.allFinite();
}

void BidirectionalEnergyFlowControl::computeTaskSpaceErrors(
    const ControlInput& input, const ControlTarget& target) {
  real_task_error_ =
      computePoseError(target, input.real_model.end_effector_transform);
  nominal_task_error_ =
      computePoseError(target, nominal_model_.end_effector_transform);
}

void BidirectionalEnergyFlowControl::computeJointTrackingErrors(
    const ControlInput& input, double period) {
  tracking_error_ = nominal_joint_position_ - input.joint_position;
  tracking_velocity_error_ = nominal_joint_velocity_ - input.joint_velocity;
  tracking_error_integral_.noalias() += tracking_error_ * period;
  for (std::size_t i = 0; i < 7; ++i) {
    const double limit = std::abs(parameters_.tracking_integral_limit[i]);
    tracking_error_integral_[i] =
        clamp(tracking_error_integral_[i], -limit, limit);
  }
}

void BidirectionalEnergyFlowControl::computeNominalControlTorque() {
  // Paper (5): tau_v = J_n^T (K * epsilon_dn - D * p_dot_n).
  // p_des is the equilibrium pose of the virtual spring.  The damper is
  // connected to the inertial frame, so it acts on the absolute nominal
  // end-effector velocity rather than on a target-velocity tracking error.
  const Vector6d wrench =
      parameters_.task_stiffness * nominal_task_error_ -
      parameters_.task_damping * nominal_model_.end_effector_velocity;
  nominal_control_torque_.noalias() = nominal_model_.jacobian.transpose() * wrench;
}

void BidirectionalEnergyFlowControl::computeCompensationTorque() {
  compensation_torque_.noalias() =
      parameters_.tracking_velocity_gain * tracking_velocity_error_ +
      parameters_.tracking_position_gain * tracking_error_ +
      parameters_.tracking_integral_gain * tracking_error_integral_;
}

void BidirectionalEnergyFlowControl::computeStorageFunctions(
    const ControlInput& input) {
  nominal_storage_ =
      0.5 * nominal_joint_velocity_.dot(nominal_model_.mass * nominal_joint_velocity_) +
      0.5 * nominal_task_error_.dot(parameters_.task_stiffness * nominal_task_error_);
  estimated_real_storage_ =
      0.5 * parameters_.inertia_scale *
          input.joint_velocity.dot(input.real_model.mass * input.joint_velocity) +
      0.5 * real_task_error_.dot(parameters_.task_stiffness * real_task_error_);
}

void BidirectionalEnergyFlowControl::computeEnergyFlowIndicators() {
  external_error_power_ = tracking_velocity_error_.dot(external_joint_torque_);
  compensation_error_power_ =
      parameters_.inertia_scale * tracking_velocity_error_.dot(compensation_torque_);
  unopposed_generation_power_ =
      (compensation_error_power_ <= 0.0 && external_error_power_ > 0.0)
          ? external_error_power_
          : 0.0;
}

void BidirectionalEnergyFlowControl::updateModulationAndTank(
    const ControlInput& input, double period) {
  if (tank_energy_ <= parameters_.tank_lower_bound) {
    alpha_raw_ = 1.0;
  } else if (external_error_power_ <= 0.0 ||
             compensation_error_power_ <= kNumericalEpsilon) {
    alpha_raw_ = 0.0;
  } else {
    alpha_raw_ = clamp(external_error_power_ / compensation_error_power_, 0.0, 1.0);
  }

  if (parameters_.filter_alpha) {
    const double coefficient =
        1.0 - std::exp(-kTwoPi * parameters_.alpha_filter_cutoff * period);
    alpha_ += coefficient * (alpha_raw_ - alpha_);
  } else {
    alpha_ = alpha_raw_;
  }
  alpha_ = clamp(alpha_, 0.0, 1.0);
  alpha_dot_ = (alpha_ - alpha_previous_) / period;

  const double nominal_dissipation =
      nominal_model_.end_effector_velocity.dot(
          parameters_.task_damping * nominal_model_.end_effector_velocity);
  const double real_dissipation = input.real_model.end_effector_velocity.dot(
      parameters_.task_damping * input.real_model.end_effector_velocity);
  tank_power_ =
      (1.0 - alpha_) * nominal_dissipation + alpha_ * real_dissipation -
      alpha_dot_ * (estimated_real_storage_ - nominal_storage_) -
      (1.0 - alpha_) * unopposed_generation_power_;

  if ((tank_energy_ < parameters_.tank_upper_bound || tank_power_ < 0.0) &&
      tank_state_ > kNumericalEpsilon) {
    tank_state_ += (tank_power_ / tank_state_) * period;
    tank_state_ = std::max(0.0, tank_state_);
  }
  tank_energy_ = clamp(0.5 * tank_state_ * tank_state_, 0.0,
                       parameters_.tank_upper_bound);
  tank_state_ = std::sqrt(2.0 * tank_energy_);
  alpha_previous_ = alpha_;
}

void BidirectionalEnergyFlowControl::computeAuxiliaryTorque(
    const ControlInput& input) {
  const double inverse_lambda = 1.0 / parameters_.inertia_scale;
  // Paper (14): K * epsilon_dr - D * p_dot.  As in the nominal controller,
  // damping is absolute end-effector velocity feedback, not velocity-error
  // feedback with respect to p_dot_des.
  const Vector6d wrench =
      parameters_.task_stiffness * real_task_error_ -
      parameters_.task_damping * input.real_model.end_effector_velocity;
  // Franka's torque interface already compensates gravity; do not duplicate
  // the explicit g(q) term in paper equation (14).
  auxiliary_torque_.noalias() =
      (inverse_lambda - 1.0) * external_joint_torque_ +
      inverse_lambda * input.real_model.jacobian.transpose() * wrench;
}

void BidirectionalEnergyFlowControl::computeBidirectionalTorque() {
  bidirectional_torque_ =
      parameters_.inertia_scale * alpha_ * compensation_torque_;
}

bool BidirectionalEnergyFlowControl::computeNullspaceTorque(
    const ControlInput& input) {
  // Dynamically consistent torque projector:
  // N_tau = I - J^T (J M^-1 J^T + mu^2 I)^-1 J M^-1.
  // The small damping mu keeps the projection bounded near singularities.
  const Eigen::LDLT<Matrix7d> mass_decomposition(input.real_model.mass);
  if (mass_decomposition.info() != Eigen::Success) {
    return false;
  }
  const Matrix76d mass_inverse_jacobian_transpose =
      mass_decomposition.solve(input.real_model.jacobian.transpose());
  if (mass_decomposition.info() != Eigen::Success ||
      !mass_inverse_jacobian_transpose.allFinite()) {
    return false;
  }

  Matrix6d operational_inverse =
      input.real_model.jacobian * mass_inverse_jacobian_transpose;
  const double damping_squared =
      parameters_.nullspace_projection_damping *
      parameters_.nullspace_projection_damping;
  operational_inverse.diagonal().array() += damping_squared;
  const Eigen::LDLT<Matrix6d> operational_decomposition(operational_inverse);
  if (operational_decomposition.info() != Eigen::Success) {
    return false;
  }

  const Matrix67d weighted_jacobian =
      mass_inverse_jacobian_transpose.transpose();
  const Matrix67d projected_task_map =
      operational_decomposition.solve(weighted_jacobian);
  if (operational_decomposition.info() != Eigen::Success ||
      !projected_task_map.allFinite()) {
    return false;
  }
  const Matrix7d torque_nullspace =
      Matrix7d::Identity() -
      input.real_model.jacobian.transpose() * projected_task_map;
  const Vector7d posture_torque =
      parameters_.nullspace_stiffness *
          (nullspace_reference_ - input.joint_position) -
      parameters_.nullspace_damping * input.joint_velocity;
  nullspace_torque_.noalias() = torque_nullspace * posture_torque;
  return nullspace_torque_.allFinite();
}

bool BidirectionalEnergyFlowControl::computeNominalAcceleration(
    const ControlInput& input, double period) {
  // Treat Cartesian damping implicitly while keeping the remaining terms
  // explicit over this control period:
  //
  //   (M + h J^T D J) dq_{k+1}
  //       = M dq_k + h [J^T(F_ext + K e) - tau_b - C].
  //
  // The previous explicit update contains the factor
  // I - h M^-1 J^T D J.  With a large task damping that factor can leave the
  // unit circle and make the simulated nominal robot diverge even though the
  // corresponding continuous-time damper is passive.
  const Matrix7d joint_damping =
      nominal_model_.jacobian.transpose() * parameters_.task_damping *
      nominal_model_.jacobian;
  const Matrix7d implicit_matrix =
      nominal_model_.mass + period * joint_damping;
  const Vector7d explicit_torque =
      nominal_model_.jacobian.transpose() *
          (input.external_wrench +
           parameters_.task_stiffness * nominal_task_error_) -
      bidirectional_torque_ - nominal_model_.coriolis;
  const Vector7d right_hand_side =
      nominal_model_.mass * nominal_joint_velocity_ +
      period * explicit_torque;

  const Eigen::LDLT<Matrix7d> decomposition(implicit_matrix);
  if (decomposition.info() != Eigen::Success) {
    return false;
  }
  const Vector7d next_velocity = decomposition.solve(right_hand_side);
  if (decomposition.info() != Eigen::Success || !next_velocity.allFinite()) {
    return false;
  }
  nominal_joint_acceleration_ =
      (next_velocity - nominal_joint_velocity_) / period;
  return nominal_joint_acceleration_.allFinite();
}

void BidirectionalEnergyFlowControl::integrateNominalRobot(double period) {
  nominal_joint_velocity_.noalias() += nominal_joint_acceleration_ * period;
  nominal_joint_position_.noalias() += nominal_joint_velocity_ * period;
}

ControlOutput BidirectionalEnergyFlowControl::computeTotalMotorTorque() const {
  ControlOutput output;
  output.total_joint_torque =
      auxiliary_torque_ + (1.0 - alpha_) * compensation_torque_ +
      nullspace_torque_;
  output.valid = output.total_joint_torque.allFinite();
  return output;
}

Vector6d BidirectionalEnergyFlowControl::computePoseError(
    const ControlTarget& target, const Eigen::Affine3d& actual_transform) {
  Vector6d error;
  error.head<3>() = target.position - actual_transform.translation();
  Eigen::Quaterniond desired = target.orientation.normalized();
  const Eigen::Quaterniond actual(actual_transform.linear());
  if (actual.coeffs().dot(desired.coeffs()) < 0.0) {
    desired.coeffs() *= -1.0;
  }
  const Eigen::Quaterniond difference = actual.inverse() * desired;
  error.tail<3>() = 2.0 * actual_transform.linear() * difference.vec();
  return error;
}

double BidirectionalEnergyFlowControl::clamp(double value, double lower,
                                             double upper) {
  return std::max(lower, std::min(value, upper));
}

}  // namespace variable_stiffness_controllers
