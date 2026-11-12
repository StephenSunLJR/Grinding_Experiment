#include "../include/bidirectional_energy_flow_control_main.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace variable_stiffness_controllers {
namespace {

template <std::size_t Size>
std::array<double, Size> toArray(
    const Eigen::Matrix<double, static_cast<int>(Size), 1>& vector) {
  std::array<double, Size> result{};
  for (std::size_t i = 0; i < Size; ++i) {
    result[i] = vector[static_cast<int>(i)];
  }
  return result;
}

bool readDiagonal(ros::NodeHandle& node_handle, const std::string& name,
                  std::size_t expected_size, std::vector<double>* values) {
  return node_handle.getParam(name, *values) &&
         values->size() == expected_size &&
         std::all_of(values->begin(), values->end(),
                     [](double value) { return std::isfinite(value); });
}

}  // namespace

bool FrankaModelProvider::compute(
    const Vector7d& joint_position, const Vector7d& joint_velocity,
    const ModelConfiguration& configuration, RobotModelState* model) const {
  if (model_handle_ == nullptr || model == nullptr ||
      !joint_position.allFinite() || !joint_velocity.allFinite()) {
    return false;
  }
  try {
    const auto q = toArray<7>(joint_position);
    const auto dq = toArray<7>(joint_velocity);
    const auto mass = model_handle_->getMass(
        q, configuration.total_inertia, configuration.total_mass,
        configuration.flange_to_com);
    const auto coriolis = model_handle_->getCoriolis(
        q, dq, configuration.total_inertia, configuration.total_mass,
        configuration.flange_to_com);
    const auto pose = model_handle_->getPose(
        franka::Frame::kEndEffector, q,
        configuration.flange_to_end_effector,
        configuration.end_effector_to_stiffness);
    const auto jacobian = model_handle_->getZeroJacobian(
        franka::Frame::kEndEffector, q,
        configuration.flange_to_end_effector,
        configuration.end_effector_to_stiffness);

    model->mass = Eigen::Map<const Matrix7d>(mass.data());
    model->coriolis = Eigen::Map<const Vector7d>(coriolis.data());
    model->jacobian = Eigen::Map<const Matrix67d>(jacobian.data());
    model->end_effector_transform =
        Eigen::Affine3d(Eigen::Map<const Eigen::Matrix4d>(pose.data()));
    model->end_effector_velocity.noalias() =
        model->jacobian * joint_velocity;
  } catch (const std::exception&) {
    return false;
  }
  return model->mass.allFinite() && model->coriolis.allFinite() &&
         model->jacobian.allFinite() &&
         model->end_effector_transform.matrix().allFinite() &&
         model->end_effector_velocity.allFinite();
}

bool BidirectionalEnergyFlowControlMain::init(
    hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  if (!loadParameters(node_handle)) {
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  auto* effort_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (model_interface == nullptr || state_interface == nullptr ||
      effort_interface == nullptr) {
    ROS_ERROR("BidirectionalEnergyFlowControl: required Franka interfaces unavailable.");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id_ + "_model"));
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id_ + "_robot"));
    effort_joint_handles_.reserve(joint_names_.size());
    for (const auto& joint_name : joint_names_) {
      effort_joint_handles_.push_back(effort_interface->getHandle(joint_name));
    }
  } catch (const hardware_interface::HardwareInterfaceException& exception) {
    ROS_ERROR_STREAM("BidirectionalEnergyFlowControl: handle error: "
                     << exception.what());
    return false;
  }

  model_provider_ =
      std::make_unique<FrankaModelProvider>(model_handle_.get());
  if (!algorithm_.configure(algorithm_parameters_, model_provider_.get())) {
    ROS_ERROR("BidirectionalEnergyFlowControl: invalid algorithm parameters.");
    return false;
  }

  diagnostics_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<BidirectionalDiagnostics>>(
      node_handle, "diagnostics", 100);
  external_wrench_buffer_.writeFromNonRT(ExternalWrenchSample{});
  target_buffer_.writeFromNonRT(TargetSample{});
  external_wrench_subscriber_ = node_handle.subscribe(
      external_wrench_topic_, 1,
      &BidirectionalEnergyFlowControlMain::externalWrenchCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  target_pose_subscriber_ = node_handle.subscribe(
      target_pose_topic_, 1,
      &BidirectionalEnergyFlowControlMain::targetPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  safe_stop_request_service_ = node_handle.advertiseService(
      "request_safe_stop",
      &BidirectionalEnergyFlowControlMain::requestSafeStop, this);
  safe_stop_status_service_ = node_handle.advertiseService(
      "safe_stop_status", &BidirectionalEnergyFlowControlMain::safeStopStatus,
      this);
  return true;
}

void BidirectionalEnergyFlowControlMain::starting(const ros::Time& time) {
  const franka::RobotState robot_state = state_handle_->getRobotState();
  ++diagnostics_run_id_;
  diagnostics_cycle_ = 0;
  diagnostics_drops_ = 0;
  last_calculated_torque_.setZero();
  last_commanded_torque_ = Eigen::Map<const Vector7d>(robot_state.tau_J_d.data());
  initial_joint_position_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  safe_return_start_position_ = initial_joint_position_;
  safe_stop_requested_.store(false, std::memory_order_release);
  safe_stop_completed_.store(false, std::memory_order_release);
  safe_return_active_ = false;
  safe_return_settled_duration_ = 0.0;
  ControlInput input;
  if (!buildControlInput(robot_state, Vector6d::Zero(), &input)) {
    algorithm_initialized_ = false;
    ROS_ERROR("BidirectionalEnergyFlowControl: invalid initial robot state.");
    return;
  }

  ControlTarget initial_target;
  initial_target.position = input.real_model.end_effector_transform.translation();
  initial_target.orientation = Eigen::Quaterniond(
      input.real_model.end_effector_transform.linear()).normalized();
  TargetSample target_sample;
  target_sample.target = initial_target;
  target_sample.stamp = time;
  target_sample.valid = true;
  target_buffer_.writeFromNonRT(target_sample);
  algorithm_initialized_ = algorithm_.initialize(input, initial_target);
  if (!algorithm_initialized_) {
    ROS_ERROR("BidirectionalEnergyFlowControl: algorithm initialization failed.");
  }
  controller_running_.store(algorithm_initialized_, std::memory_order_release);
}

void BidirectionalEnergyFlowControlMain::update(
    const ros::Time& time, const ros::Duration& period) {
  const franka::RobotState robot_state = state_handle_->getRobotState();
  const Vector7d previous_torque =
      Eigen::Map<const Vector7d>(robot_state.tau_J_d.data());

  if (safe_stop_requested_.load(std::memory_order_acquire)) {
    updateSafeReturn(robot_state, time, period, previous_torque);
    publishDiagnostics(time, period, robot_state, external_wrench_buffer_.readFromRT(), 5, false);
    return;
  }

  const ExternalWrenchSample* wrench_sample =
      external_wrench_buffer_.readFromRT();
  const bool wrench_fresh =
      wrench_sample != nullptr && wrench_sample->valid &&
      (time - wrench_sample->stamp).toSec() <= external_wrench_timeout_;
  if (require_external_wrench_ && !wrench_fresh) {
    writeTorqueCommand(previous_torque);
    publishDiagnostics(time, period, robot_state, wrench_sample, 1, false);
    return;
  }
  const Vector6d wrench = wrench_fresh ? wrench_sample->wrench : Vector6d::Zero();

  ControlInput input;
  if (!algorithm_initialized_ || !buildControlInput(robot_state, wrench, &input)) {
    writeTorqueCommand(previous_torque);
    publishDiagnostics(time, period, robot_state, wrench_sample, 2, false);
    return;
  }
  const TargetSample* target_sample = target_buffer_.readFromRT();
  if (target_sample == nullptr || !target_sample->valid) {
    writeTorqueCommand(previous_torque);
    publishDiagnostics(time, period, robot_state, wrench_sample, 3, false);
    return;
  }

  const ControlOutput output = algorithm_.computeControl(
      input, target_sample->target, period.toSec());
  if (!output.valid) {
    writeTorqueCommand(previous_torque);
    publishDiagnostics(time, period, robot_state, wrench_sample, 4, false);
    return;
  }
  last_calculated_torque_ = output.total_joint_torque;
  writeTorqueCommand(
      applyTorqueSafety(output.total_joint_torque, previous_torque));
  publishDiagnostics(time, period, robot_state, wrench_sample, 0, true);
}

void BidirectionalEnergyFlowControlMain::stopping(const ros::Time& /*time*/) {
  controller_running_.store(false, std::memory_order_release);
}

bool BidirectionalEnergyFlowControlMain::loadParameters(
    ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_) ||
      !node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != 7) {
    ROS_ERROR("BidirectionalEnergyFlowControl: invalid arm_id or joint_names.");
    return false;
  }

  node_handle.param("external_wrench_topic", external_wrench_topic_,
                    external_wrench_topic_);
  node_handle.param("external_wrench_frame", external_wrench_frame_,
                    external_wrench_frame_);
  node_handle.param("external_wrench_in_end_effector_frame",
                    external_wrench_in_end_effector_frame_,
                    external_wrench_in_end_effector_frame_);
  node_handle.param("target_pose_topic", target_pose_topic_, target_pose_topic_);
  node_handle.param("target_pose_frame", target_pose_frame_,
                    target_pose_frame_);
  node_handle.param("external_wrench_timeout", external_wrench_timeout_,
                    external_wrench_timeout_);
  node_handle.param("external_wrench_sign", external_wrench_sign_,
                    external_wrench_sign_);
  node_handle.param("require_external_wrench", require_external_wrench_,
                    require_external_wrench_);
  node_handle.param("maximum_torque_step", maximum_torque_step_,
                    maximum_torque_step_);
  node_handle.param("lambda", algorithm_parameters_.inertia_scale, 1.0);
  node_handle.param("tank_initial_energy",
                    algorithm_parameters_.tank_initial_energy, 1.0);
  node_handle.param("tank_lower_bound",
                    algorithm_parameters_.tank_lower_bound, 0.1);
  node_handle.param("tank_upper_bound",
                    algorithm_parameters_.tank_upper_bound, 1.0);
  node_handle.param("filter_alpha", algorithm_parameters_.filter_alpha, false);
  node_handle.param("alpha_filter_cutoff",
                    algorithm_parameters_.alpha_filter_cutoff, 20.0);

  std::vector<double> stiffness, damping, k_gamma, k_p, k_i;
  std::vector<double> nullspace_stiffness, nullspace_damping;
  std::vector<double> integral_limit, torque_limit;
  std::vector<double> return_kp, return_kd;
  if (!readDiagonal(node_handle, "task_stiffness", 6, &stiffness) ||
      !readDiagonal(node_handle, "task_damping", 6, &damping) ||
      !readDiagonal(node_handle, "tracking_velocity_gain", 7, &k_gamma) ||
      !readDiagonal(node_handle, "tracking_position_gain", 7, &k_p) ||
      !readDiagonal(node_handle, "tracking_integral_gain", 7, &k_i) ||
      !readDiagonal(node_handle, "tracking_integral_limit", 7,
                    &integral_limit) ||
      !readDiagonal(node_handle, "nullspace_stiffness", 7,
                    &nullspace_stiffness) ||
      !readDiagonal(node_handle, "nullspace_damping", 7,
                    &nullspace_damping) ||
      !readDiagonal(node_handle, "absolute_torque_limit", 7, &torque_limit) ||
      !readDiagonal(node_handle, "safe_return_position_gain", 7, &return_kp) ||
      !readDiagonal(node_handle, "safe_return_velocity_gain", 7, &return_kd)) {
    ROS_ERROR("BidirectionalEnergyFlowControl: invalid gain or limit vector.");
    return false;
  }
  for (std::size_t i = 0; i < 6; ++i) {
    algorithm_parameters_.task_stiffness(i, i) = stiffness[i];
    algorithm_parameters_.task_damping(i, i) = damping[i];
  }
  for (std::size_t i = 0; i < 7; ++i) {
    algorithm_parameters_.tracking_velocity_gain(i, i) = k_gamma[i];
    algorithm_parameters_.tracking_position_gain(i, i) = k_p[i];
    algorithm_parameters_.tracking_integral_gain(i, i) = k_i[i];
    algorithm_parameters_.tracking_integral_limit[i] = integral_limit[i];
    algorithm_parameters_.nullspace_stiffness(i, i) = nullspace_stiffness[i];
    algorithm_parameters_.nullspace_damping(i, i) = nullspace_damping[i];
    absolute_torque_limit_[i] = std::abs(torque_limit[i]);
    safe_return_position_gain_[i] = return_kp[i];
    safe_return_velocity_gain_[i] = return_kd[i];
  }
  node_handle.param("safe_return_duration", safe_return_duration_,
                    safe_return_duration_);
  node_handle.param("nullspace_projection_damping",
                    algorithm_parameters_.nullspace_projection_damping, 0.01);
  node_handle.param("safe_return_position_tolerance",
                    safe_return_position_tolerance_,
                    safe_return_position_tolerance_);
  node_handle.param("safe_return_velocity_tolerance",
                    safe_return_velocity_tolerance_,
                    safe_return_velocity_tolerance_);
  node_handle.param("safe_return_settle_time", safe_return_settle_time_,
                    safe_return_settle_time_);
  return external_wrench_timeout_ > 0.0 &&
         std::abs(external_wrench_sign_) > 0.0 &&
         maximum_torque_step_ > 0.0 && safe_return_duration_ > 0.0 &&
         safe_return_position_tolerance_ > 0.0 &&
         safe_return_velocity_tolerance_ > 0.0 &&
         safe_return_settle_time_ >= 0.0 &&
         (safe_return_position_gain_.array() >= 0.0).all() &&
         (safe_return_velocity_gain_.array() >= 0.0).all();
}

void BidirectionalEnergyFlowControlMain::updateSafeReturn(
    const franka::RobotState& robot_state, const ros::Time& time,
    const ros::Duration& period, const Vector7d& previous_torque) {
  const Vector7d joint_position =
      Eigen::Map<const Vector7d>(robot_state.q.data());
  const Vector7d joint_velocity =
      Eigen::Map<const Vector7d>(robot_state.dq.data());

  if (!safe_return_active_) {
    safe_return_active_ = true;
    safe_return_start_time_ = time;
    safe_return_start_position_ = joint_position;
    safe_return_settled_duration_ = 0.0;
    safe_stop_completed_.store(false, std::memory_order_release);
  }

  const double elapsed = std::max(0.0, (time - safe_return_start_time_).toSec());
  const double normalized_time =
      std::min(1.0, elapsed / safe_return_duration_);
  const double u2 = normalized_time * normalized_time;
  const double u3 = u2 * normalized_time;
  const double u4 = u3 * normalized_time;
  const double u5 = u4 * normalized_time;
  const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  const double blend_rate =
      (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / safe_return_duration_;
  const Vector7d displacement =
      initial_joint_position_ - safe_return_start_position_;
  const Vector7d desired_position =
      safe_return_start_position_ + blend * displacement;
  const Vector7d desired_velocity = blend_rate * displacement;

  Vector7d coriolis;
  try {
    const auto coriolis_array = model_handle_->getCoriolis();
    coriolis = Eigen::Map<const Vector7d>(coriolis_array.data());
  } catch (const std::exception&) {
    writeTorqueCommand(previous_torque);
    return;
  }

  const Vector7d return_torque =
      coriolis +
      safe_return_position_gain_.cwiseProduct(desired_position - joint_position) +
      safe_return_velocity_gain_.cwiseProduct(desired_velocity - joint_velocity);
  if (!return_torque.allFinite()) {
    writeTorqueCommand(previous_torque);
    return;
  }
  writeTorqueCommand(applyTorqueSafety(return_torque, previous_torque));

  const bool trajectory_finished = normalized_time >= 1.0;
  const bool position_reached =
      (initial_joint_position_ - joint_position).cwiseAbs().maxCoeff() <=
      safe_return_position_tolerance_;
  const bool velocity_settled = joint_velocity.cwiseAbs().maxCoeff() <=
                                safe_return_velocity_tolerance_;
  if (trajectory_finished && position_reached && velocity_settled) {
    safe_return_settled_duration_ += std::max(0.0, period.toSec());
  } else {
    safe_return_settled_duration_ = 0.0;
  }
  if (safe_return_settled_duration_ >= safe_return_settle_time_) {
    safe_stop_completed_.store(true, std::memory_order_release);
  }
}

bool BidirectionalEnergyFlowControlMain::requestSafeStop(
    std_srvs::Trigger::Request& /*request*/,
    std_srvs::Trigger::Response& response) {
  if (!controller_running_.load(std::memory_order_acquire)) {
    response.success = false;
    response.message = "Controller is not running; safe return cannot start.";
    return true;
  }
  safe_stop_requested_.store(true, std::memory_order_release);
  response.success = true;
  response.message = "Safe return requested; keep the controller running.";
  return true;
}

bool BidirectionalEnergyFlowControlMain::safeStopStatus(
    std_srvs::Trigger::Request& /*request*/,
    std_srvs::Trigger::Response& response) {
  response.success = safe_stop_completed_.load(std::memory_order_acquire);
  if (response.success) {
    response.message = "Initial joint position reached and settled.";
  } else if (safe_stop_requested_.load(std::memory_order_acquire)) {
    response.message = "Safe return is in progress.";
  } else if (!controller_running_.load(std::memory_order_acquire)) {
    response.message = "Controller is not running.";
  } else {
    response.message = "Safe return has not been requested.";
  }
  return true;
}

bool BidirectionalEnergyFlowControlMain::buildControlInput(
    const franka::RobotState& robot_state, const Vector6d& external_wrench,
    ControlInput* input) const {
  if (input == nullptr) {
    return false;
  }
  input->joint_position = Eigen::Map<const Vector7d>(robot_state.q.data());
  input->joint_velocity = Eigen::Map<const Vector7d>(robot_state.dq.data());
  input->previous_desired_torque =
      Eigen::Map<const Vector7d>(robot_state.tau_J_d.data());
  input->model_configuration.total_inertia = robot_state.I_total;
  input->model_configuration.total_mass = robot_state.m_total;
  input->model_configuration.flange_to_com = robot_state.F_x_Ctotal;
  input->model_configuration.flange_to_end_effector = robot_state.F_T_EE;
  input->model_configuration.end_effector_to_stiffness = robot_state.EE_T_K;
  if (!model_provider_->compute(input->joint_position, input->joint_velocity,
                                input->model_configuration,
                                &input->real_model)) {
    return false;
  }

  if (external_wrench_in_end_effector_frame_) {
    // The sensor node has already shifted the wrench to the same end-effector
    // origin used by kEndEffector. Rotate force and moment into the base axes;
    // no translation term is allowed here because the wrench application point
    // must remain the end-effector origin used by the Jacobian.
    const Eigen::Matrix3d rotation =
        input->real_model.end_effector_transform.linear();
    input->external_wrench.head<3>().noalias() =
        rotation * external_wrench.head<3>();
    input->external_wrench.tail<3>().noalias() =
        rotation * external_wrench.tail<3>();
  } else {
    input->external_wrench = external_wrench;
  }
  return input->external_wrench.allFinite();
}

Vector7d BidirectionalEnergyFlowControlMain::applyTorqueSafety(
    const Vector7d& calculated, const Vector7d& previous) const {
  Vector7d safe;
  for (std::size_t i = 0; i < 7; ++i) {
    const double absolute_limited =
        std::max(-absolute_torque_limit_[i],
                 std::min(calculated[i], absolute_torque_limit_[i]));
    const double difference = absolute_limited - previous[i];
    safe[i] = previous[i] +
              std::max(-maximum_torque_step_,
                       std::min(difference, maximum_torque_step_));
  }
  return safe;
}

void BidirectionalEnergyFlowControlMain::writeTorqueCommand(
    const Vector7d& torque) {
  if (!torque.allFinite() || effort_joint_handles_.size() != 7) {
    return;
  }
  last_commanded_torque_ = torque;
  for (std::size_t i = 0; i < 7; ++i) {
    effort_joint_handles_[i].setCommand(torque[i]);
  }
}


void BidirectionalEnergyFlowControlMain::publishDiagnostics(
    const ros::Time& time, const ros::Duration& period,
    const franka::RobotState& state, const ExternalWrenchSample* wrench,
    uint8_t status, bool diagnostics_valid) {
  ++diagnostics_cycle_;
  if (!diagnostics_publisher_ || !diagnostics_publisher_->trylock()) {
    ++diagnostics_drops_;
    return;
  }
  auto& msg = diagnostics_publisher_->msg_;
  msg.header.stamp = time;
  msg.run_id = diagnostics_run_id_;
  msg.cycle = diagnostics_cycle_;
  msg.publish_drops = diagnostics_drops_;
  msg.status = status;
  msg.period = period.toSec();
  msg.wrench_valid = wrench && wrench->valid;
  msg.wrench_age = msg.wrench_valid ? (time - wrench->stamp).toSec() : -1.0;
  msg.wrench_fresh = msg.wrench_valid && msg.wrench_age <= external_wrench_timeout_;
  msg.diagnostics_valid = diagnostics_valid;
  const auto& d = algorithm_.diagnostics();
  for (std::size_t i = 0; i < 7; ++i) {
    msg.q[i] = state.q[i]; msg.dq[i] = state.dq[i];
    msg.measured_torque[i] = state.tau_J[i];
    msg.estimated_external_joint_torque[i] = state.tau_ext_hat_filtered[i];
    msg.previous_torque[i] = state.tau_J_d[i];
    msg.calculated_torque[i] = last_calculated_torque_[i];
    msg.commanded_torque[i] = last_commanded_torque_[i];
    msg.nominal_q[i] = d.nominal_joint_position[i];
    msg.nominal_dq[i] = d.nominal_joint_velocity[i];
    msg.nominal_ddq[i] = d.nominal_joint_acceleration[i];
    msg.external_joint_torque[i] = d.external_joint_torque[i];
    msg.nominal_control_torque[i] = d.nominal_control_torque[i];
    msg.compensation_torque[i] = d.compensation_torque[i];
    msg.auxiliary_torque[i] = d.auxiliary_torque[i];
    msg.bidirectional_torque[i] = d.bidirectional_torque[i];
    msg.nullspace_torque[i] = d.nullspace_torque[i];
  }
  for (std::size_t i = 0; i < 6; ++i) {
    msg.wrench[i] = msg.wrench_valid ? wrench->wrench[i] : 0.0;
    msg.real_task_error[i] = d.real_task_error[i];
    msg.nominal_task_error[i] = d.nominal_task_error[i];
  }
  msg.alpha_raw = d.alpha_raw; msg.alpha = d.alpha; msg.alpha_dot = d.alpha_dot;
  msg.nominal_storage = d.nominal_storage;
  msg.estimated_real_storage = d.estimated_real_storage;
  msg.tank_energy = d.tank_energy; msg.tank_power = d.tank_power;
  msg.external_error_power = d.external_error_power;
  msg.compensation_error_power = d.compensation_error_power;
  diagnostics_publisher_->unlockAndPublish();
}

void BidirectionalEnergyFlowControlMain::externalWrenchCallback(
    const geometry_msgs::WrenchStampedConstPtr& message) {
  ExternalWrenchSample sample;
  sample.stamp = message->header.stamp.isZero() ? ros::Time::now()
                                                : message->header.stamp;
  if (!external_wrench_frame_.empty() &&
      message->header.frame_id != external_wrench_frame_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "BidirectionalEnergyFlowControl: expected wrench frame '"
                 << external_wrench_frame_ << "' but received '"
                 << message->header.frame_id << "'.");
    external_wrench_buffer_.writeFromNonRT(sample);
    return;
  }
  sample.wrench << message->wrench.force.x, message->wrench.force.y,
      message->wrench.force.z, message->wrench.torque.x,
      message->wrench.torque.y, message->wrench.torque.z;
  sample.wrench *= external_wrench_sign_;
  sample.valid = sample.wrench.allFinite();
  external_wrench_buffer_.writeFromNonRT(sample);
}

void BidirectionalEnergyFlowControlMain::targetPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& message) {
  TargetSample sample;
  sample.stamp = message->header.stamp.isZero() ? ros::Time::now()
                                                : message->header.stamp;
  if (!target_pose_frame_.empty() &&
      message->header.frame_id != target_pose_frame_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "BidirectionalEnergyFlowControl: expected target frame '"
                 << target_pose_frame_ << "' but received '"
                 << message->header.frame_id << "'.");
    return;
  }
  sample.target.position << message->pose.position.x,
      message->pose.position.y, message->pose.position.z;
  sample.target.orientation = Eigen::Quaterniond(
      message->pose.orientation.w, message->pose.orientation.x,
      message->pose.orientation.y, message->pose.orientation.z);
  if (sample.target.orientation.norm() > 1.0e-9) {
    sample.target.orientation.normalize();
    sample.valid = sample.target.position.allFinite() &&
                   sample.target.orientation.coeffs().allFinite();
  }
  if (sample.valid) {
    target_buffer_.writeFromNonRT(sample);
  }
}

}  // namespace variable_stiffness_controllers

PLUGINLIB_EXPORT_CLASS(
    variable_stiffness_controllers::BidirectionalEnergyFlowControlMain,
    controller_interface::ControllerBase)
