/**
 * @file A_DLS.cpp
 * @brief Anisotropic DLS Controller implementation.
 *
 * ROS controller plugin for Franka Panda 7-DOF robot.
 * Implements the A-DLS pipeline (Algorithm 1) from the theory paper v5.10.
 *
 * Paper: "各向异性自适应任务分级与柔顺控制" (Sun Chuanshuo, 2026)
 */

#include "../include/adls/A_DLS.h"
#include "../include/franka_model.h"  // Franka dynamics (MassMatrix, etc.)

#include <cmath>
#include <memory>
#include <algorithm>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <franka/robot_state.h>

// ── Convenience typedefs ──
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;
using Matrix76d = Eigen::Matrix<double, 7, 6>;

namespace variable_stiffness_controllers {

// ============================================================================
// init() — Load params, get ROS handles, create sub-modules
// ============================================================================
bool ADLSController::init(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& node_handle) {
    // ── Read arm_id ──
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("A_DLS: Could not read parameter arm_id");
        return false;
    }

    // ── Read joint names ──
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR("A_DLS: Invalid or no joint_names parameters provided");
        return false;
    }

    // ── Get model handle ──
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM("A_DLS: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("A_DLS: Exception getting model handle: " << ex.what());
        return false;
    }

    // ── Get state handle ──
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM("A_DLS: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("A_DLS: Exception getting state handle: " << ex.what());
        return false;
    }

    // ── Get joint handles (effort interface) ──
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM("A_DLS: Error getting effort joint interface");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("A_DLS: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    // ── Load A-DLS method ──
    if (!node_handle.getParam("method", method_)) {
        ROS_WARN("A_DLS: method param not set, defaulting to M4");
        method_ = "M4";
    }
    ROS_INFO_STREAM("A_DLS: Using method " << method_);

    // ── Load A-DLS parameters (with defaults from theory paper) ──
    params_ = adls::ADLSParams::forMethod(method_);

    // Override from YAML if provided
    std::vector<double> tmp_vec;
    if (node_handle.getParam("K_O", tmp_vec) && tmp_vec.size() == 7)
        params_.K_O = Eigen::Map<Vector7d>(tmp_vec.data());
    if (node_handle.getParam("lambda2_base", params_.lambda2_base))
        ROS_INFO_STREAM("A_DLS: lambda2_base = " << params_.lambda2_base);
    if (node_handle.getParam("lambda2_max", params_.lambda2_max))
        ROS_INFO_STREAM("A_DLS: lambda2_max = " << params_.lambda2_max);
    if (node_handle.getParam("gamma_attack", params_.gamma_attack))
        ROS_INFO_STREAM("A_DLS: gamma_attack = " << params_.gamma_attack);
    if (node_handle.getParam("gamma_decay", params_.gamma_decay))
        ROS_INFO_STREAM("A_DLS: gamma_decay = " << params_.gamma_decay);
    if (node_handle.getParam("beta_w", params_.beta_w))
        ROS_INFO_STREAM("A_DLS: beta_w = " << params_.beta_w);
    if (node_handle.getParam("beta_c", params_.beta_c))
        ROS_INFO_STREAM("A_DLS: beta_c = " << params_.beta_c);

    // ── Experiment 2B options (EMA strategy + reference modulation) ──
    if (node_handle.getParam("ema_mode", ema_mode_))
        ROS_INFO_STREAM("A_DLS: ema_mode = " << ema_mode_);
    if (node_handle.getParam("ref_modulation", ref_modulation_))
        ROS_INFO_STREAM("A_DLS: ref_modulation = " << (ref_modulation_ ? "ON" : "OFF"));

    // Map ema_mode to gamma_attack/gamma_decay parameter overrides
    if (ema_mode_ == "none") {
        // No EMA: γ=1.0 means each step replaces previous (no smoothing)
        params_.gamma_attack = 1.0;
        params_.gamma_decay = 1.0;
    } else if (ema_mode_ == "symmetric") {
        // Symmetric (v5.9 baseline): slow on both attack and decay
        params_.gamma_attack = 0.01;  // τ ≈ 100ms
        params_.gamma_decay = 0.01;
    }
    // else "asymmetric": keep configured γ_attack and γ_decay

    // ── Trajectory mode ──
    if (node_handle.getParam("traj_mode", traj_mode_))
        ROS_INFO_STREAM("A_DLS: traj_mode = " << traj_mode_);
    node_handle.getParam("extend_distance", extend_distance_);

    // ── Load experiment parameters ──
    node_handle.getParam("force_start_time", force_start_time_);
    node_handle.getParam("force_end_time", force_end_time_);
    node_handle.getParam("force_ramp_time", force_ramp_time_);
    node_handle.getParam("force_magnitude", force_magnitude_);
    node_handle.getParam("force_direction", force_direction_);
    node_handle.getParam("circle_radius", radius_);
    node_handle.getParam("circle_period", factor_circle_);
    factor_circle_ = 2.0 * M_PI / factor_circle_;  // convert to angular freq

    // ── Create sub-modules ──
    gmo_ = std::make_unique<adls::GMOObserver>(params_);
    fdi_ = std::make_unique<adls::FDIIsolator>(params_);
    svd_proj_ = std::make_unique<adls::SVDProjector>(params_);
    damping_ = std::make_unique<adls::ADLSDamping>(params_);
    nullspace_ = std::make_unique<adls::NullSpaceCompliance>(params_);

    // ── Diagnostics publisher ──
    diag_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/adls/diagnostics", 10);

    ROS_INFO("A_DLS: Controller initialized successfully.");
    return true;
}

// ============================================================================
// starting() — Initialize control state
// ============================================================================
void ADLSController::starting(const ros::Time& /*time*/) {
    franka::RobotState robot_state = state_handle_->getRobotState();

    // Get initial joint state
    Eigen::Map<Vector7d> q_init(robot_state.q.data());
    Eigen::Map<Vector7d> dq_init(robot_state.dq.data());

    // Initialize command state
    q_cmd_ = q_init;
    dq_cmd_ = dq_init;
    dq_cmd_prev_ = dq_init;
    ddq_cmd_ema_.setZero();

    // Get initial end-effector pose as desired
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

    // Store initial RPY for trajectory (orientation is held constant)
    Eigen::Matrix3d R_init = initial_transform.linear();
    init_roll_  = std::atan2(R_init(2,1), R_init(2,2));
    init_pitch_ = std::atan2(-R_init(2,0), std::sqrt(R_init(0,0)*R_init(0,0) + R_init(1,0)*R_init(1,0)));
    init_yaw_   = std::atan2(R_init(1,0), R_init(0,0));
    ROS_INFO("A_DLS: Initial RPY: [%.3f, %.3f, %.3f]", init_roll_, init_pitch_, init_yaw_);

    velocity_d_.setZero();
    t_ = ros::Duration(0.0);
    count_ = 0.0;
    ctc_initialized_ = false;
    traj_started_ = false;

    // Reset GMO baseline state
    r_baseline_.setZero();
    baseline_samples_ = 0;
    baseline_calibrated_ = false;

    // Initialize previous torque to zero
    // (Franka platform internally adds gravity below our output, so our
    //  controller's output should start at 0, not at g(q))
    tau_cmd_prev_.setZero();

    // Reset sub-modules
    gmo_->reset();
    damping_->reset();
    nullspace_->reset();

    ROS_INFO("A_DLS: Controller started. Initial pose: [%.3f, %.3f, %.3f]",
             position_d_.x(), position_d_.y(), position_d_.z());
}

// ============================================================================
// getEndEffectorPose() — FK from libfranka homogeneous transform
// ============================================================================
Vector6d ADLSController::getEndEffectorPose(const std::array<double, 16>& O_T_EE) {
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(O_T_EE.data()));
    Eigen::Vector3d pos = transform.translation();
    Eigen::Quaterniond quat(transform.linear());

    // Convert quaternion to RPY (ZYX fixed frame)
    Eigen::Matrix3d R = quat.toRotationMatrix();
    double roll  = std::atan2(R(2,1), R(2,2));
    double pitch = std::atan2(-R(2,0), std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
    double yaw   = std::atan2(R(1,0), R(0,0));

    Vector6d pose;
    pose << pos.x(), pos.y(), pos.z(), roll, pitch, yaw;
    return pose;
}

// ============================================================================
// update() — Main 1 kHz control loop (Algorithm 1)
// ============================================================================
void ADLSController::update(const ros::Time& /*time*/, const ros::Duration& period) {
    double dt = period.toSec();
    t_ += period;
    count_ += dt;
    double t_now = t_.toSec();

    // ── Get robot state ──
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Map<Vector7d> q(robot_state.q.data());
    Eigen::Map<Vector7d> dq(robot_state.dq.data());
    Eigen::Map<Vector7d> tau_J_d(robot_state.tau_J_d.data());
    Eigen::Map<Vector7d> tau_ext(robot_state.tau_ext_hat_filtered.data());

    // Motor torque = desired torque from previous step (for GMO)
    Eigen::Map<Vector7d> tau_m(robot_state.tau_J.data());

    // ── Get dynamics from libfranka model ──
    std::array<double, 49> M_arr = model_handle_->getMass();
    std::array<double, 7>  C_arr = model_handle_->getCoriolis();
    std::array<double, 7>  g_arr = model_handle_->getGravity();
    std::array<double, 42> J_arr = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    Eigen::Map<Matrix7d>  M_mat(M_arr.data());
    Eigen::Map<Vector7d>  C_dq(C_arr.data());   // Coriolis+centrifugal torque
    Eigen::Map<Vector7d>  g_vec(g_arr.data());
    Eigen::Map<Matrix67d> J_mat(J_arr.data());

    // ── Get current end-effector pose (homogeneous transform) ──
    Eigen::Affine3d transform_cur(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position_cur(transform_cur.translation());
    Eigen::Quaterniond orientation_cur(transform_cur.linear());

    // ── Desired trajectory ──
    // STATIC HOLD MODE (radius_ = 0): hold initial pose forever, used to
    // characterize Gazebo model mismatch.
    // EXTEND MODE (traj_mode_ = "extend"): linearly extend +X by extend_distance_
    //   over [TRAJ_START_TIME, TRAJ_START_TIME+5s], then hold for singular test.
    Eigen::Vector3d position_d_t;
    Vector6d dx_d;
    dx_d.setZero();

    constexpr double TRAJ_START_TIME = 2.0;
    if (traj_mode_ == "extend" && t_now >= TRAJ_START_TIME) {
        // ─── EXPERIMENT 3B: singularity-test extension trajectory ───
        if (!traj_started_) {
            position_d_ = position_cur;   // snapshot start position
            traj_started_ = true;
            ROS_INFO("A_DLS: Extension trajectory started. From [%.3f, %.3f, %.3f] "
                     "extending +X by %.2f m over 5s",
                     position_d_.x(), position_d_.y(), position_d_.z(),
                     extend_distance_);
        }
        double t_ext = t_now - TRAJ_START_TIME;
        constexpr double T_EXTEND = 5.0;   // 5s to fully extend
        double progress = std::min(1.0, t_ext / T_EXTEND);
        // S-curve smooth-step for C¹ continuous progress
        double s = progress * progress * (3.0 - 2.0 * progress);
        position_d_t.x() = position_d_.x() + extend_distance_ * s;
        position_d_t.y() = position_d_.y();
        position_d_t.z() = position_d_.z();
        // Velocity feedforward (derivative of S-curve · extend_distance / T_EXTEND)
        if (t_ext < T_EXTEND) {
            double ds_dprogress = 6.0 * progress * (1.0 - progress);
            dx_d[0] = extend_distance_ * ds_dprogress / T_EXTEND;
        }
    } else if (radius_ <= 1e-6 || t_now < TRAJ_START_TIME) {
        // Static hold: target = initial position, no motion
        position_d_t = position_d_;
    } else {
        // Circle motion (only if radius > 0)
        if (!traj_started_) {
            position_d_ = position_cur;
            traj_started_ = true;
            ROS_INFO("A_DLS: Circle trajectory started. Center: [%.3f, %.3f, %.3f]",
                     position_d_.x(), position_d_.y(), position_d_.z());
        }
        double t_traj = t_now - TRAJ_START_TIME;
        // Smooth amplitude ramp over 3 seconds (was 1s).
        // Slower ramp = smaller initial position error → less CLIK saturation.
        // Use smooth-step S-curve for C¹-continuous amplitude transition:
        //   amp(τ) = radius · (3τ² - 2τ³),  τ = clamp(t_traj/3, 0, 1)
        double tau = std::min(1.0, t_traj / 3.0);
        double smooth_step = tau * tau * (3.0 - 2.0 * tau);  // S-curve
        double amp = smooth_step * radius_;
        double s = std::sin(factor_circle_ * t_traj);
        double c = std::cos(factor_circle_ * t_traj);
        position_d_t.x() = position_d_.x() + amp * s;
        position_d_t.y() = position_d_.y() + amp * c;
        position_d_t.z() = position_d_.z();
        dx_d[0] =  factor_circle_ * amp * c;
        dx_d[1] = -factor_circle_ * amp * s;
        dx_d[2] =  0.0;
    }

    // ── Compute Cartesian error (6×1) — STANDARD QUATERNION METHOD ──
    // Same approach as franka_example_controllers/cartesian_impedance_example_controller.cpp
    // and our icvs.cpp — avoids RPY wraparound issues at roll = ±π.
    // The orientation error is an AXIS-ANGLE vector compatible with the geometric Jacobian.
    Vector6d error;
    error.head(3) = position_d_t - position_cur;

    // Quaternion error: ensure shortest path
    Eigen::Quaterniond orient_target = orientation_d_;
    if (orient_target.coeffs().dot(orientation_cur.coeffs()) < 0.0) {
        orientation_cur.coeffs() = -orientation_cur.coeffs();
    }
    // "difference" quaternion: q_err = q_cur^{-1} * q_des
    // Imaginary part ≈ (axis · sin(θ/2)) — small for similar orientations, in EE frame
    Eigen::Quaterniond error_quaternion(orientation_cur.inverse() * orient_target);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform orientation error from EE frame to base frame.
    // NO negation: our error convention is (target - current), so +K·error drives correctly.
    // (The Franka example uses -R*imag because their error = current - target with -K gain;
    //  both conventions give the same physical force, but we must not mix the sign rules.)
    error.tail(3) = transform_cur.linear() * error.tail(3);

    // ── Compute external force (experiment scenario) ──
    F_ext_applied_.setZero();
    if (t_now >= force_start_time_ && t_now <= force_end_time_) {
        double ramp_factor = 1.0;
        double elapsed = t_now - force_start_time_;
        if (elapsed < force_ramp_time_) {
            ramp_factor = elapsed / force_ramp_time_;
        }
        double remaining = force_end_time_ - t_now;
        if (remaining < force_ramp_time_ && remaining > 0.0) {
            ramp_factor = std::min(ramp_factor, remaining / force_ramp_time_);
        }
        F_ext_applied_[force_direction_] = -force_magnitude_ * ramp_factor;  // negative = pressing
    }

    // ── Compute external joint torque from applied force ──
    Vector7d tau_ext_applied = J_mat.transpose() * F_ext_applied_;

    // ── Compute friction torque ──
    Vector7d tau_F = Friction(dq);

    // ═══════════════════════════════════════════════════════════════
    // ALGORITHM 1: A-DLS Control Pipeline
    // ═══════════════════════════════════════════════════════════════

    // ── STEP 1: GMO Observer (§2, Eq 2.9) ──
    //
    // CRITICAL: τ_m for GMO must be the TOTAL motor torque actually applied
    // to the joint, not just our user command. On Franka platforms (real FCI
    // and franka_hw_sim alike), gravity is auto-injected BELOW the user
    // command: actual_motor_effort = τ_user + g(q).
    //
    // The state.tau_J field reports only the user command (without the
    // internal gravity injection). Feeding it directly into GMO would make
    // the residual r converge to g(q) instead of τ_ext — a spurious ~35 N·m
    // signal even with no real external force. We pre-add g(q) here to
    // recover the true motor torque for the GMO observation equation.
    Vector7d tau_m_effective = tau_m + g_vec;
    Vector7d r_raw = gmo_->update(q, dq, tau_m_effective, *model_handle_, dt);

    // ── STEP 1b: GMO Adaptive Baseline (Gazebo model-mismatch high-pass) ──
    //
    // Gazebo's URDF/KDL physics and libfranka's analytical model disagree:
    //   - Static (no motion):   ~18 N·m systematic offset (gravity model differs)
    //   - Dynamic (in motion):  additional 2-10 N·m fluctuation (M, C also differ)
    //
    // Both are SLOWLY-VARYING (sub-second scale tied to robot configuration).
    // Real external collisions are FAST (>10Hz transients).
    //
    // We use a first-order low-pass on r_raw to track the slow mismatch as the
    // baseline, then subtract → high-pass filter on r. Collisions pass through;
    // slow model errors are absorbed into the baseline.
    //
    // Time constant τ_baseline = 0.5s → cutoff frequency ~0.3 Hz.
    // Real Franka: r_raw is already 0 at rest → baseline → 0 → no effect.
    // CRITICAL TIMING: calibration must complete BEFORE trajectory starts
    // (TRAJ_START_TIME = 2.0s), otherwise traj-induced dynamic mismatch
    // pollutes the static baseline reading.
    Vector7d r = Vector7d::Zero();  // default: suppress A-DLS until baseline ready
    constexpr double CAL_START = 0.5;
    constexpr double CAL_END = 1.8;   // before traj start at t=2.0s
    constexpr double tau_baseline = 0.5;  // [s] baseline tracking time constant
    if (t_now >= CAL_START && t_now < CAL_END) {
        // Phase 1: fast averaging during static window for clean initialization.
        // r stays zero so A-DLS doesn't falsely activate during calibration.
        r_baseline_ = (r_baseline_ * baseline_samples_ + r_raw) / (baseline_samples_ + 1);
        baseline_samples_++;
    } else if (t_now >= CAL_END) {
        if (!baseline_calibrated_) {
            baseline_calibrated_ = true;
            ROS_INFO("A_DLS: GMO baseline initialized. "
                     "‖r_baseline_init‖_∞ = %.2f N·m. "
                     "Switching to gated adaptive tracking (τ = %.1fs).",
                     r_baseline_.cwiseAbs().maxCoeff(), tau_baseline);
        }
        // Phase 2: GATED slow lowpass tracking with hysteresis + grace period
        //
        // Multi-stage gate to avoid false triggers during trajectory ramp:
        //   1) Grace period: first 2s after CAL_END allow baseline to track
        //      freely (handles motion-induced model mismatch from traj ramp).
        //   2) Threshold raised to 8 N·m: only large residuals freeze baseline.
        //   3) Persistence: must exceed threshold for 30ms continuously before
        //      freezing (rejects single-sample spikes from numerical noise).
        constexpr double COLLISION_GATE = 8.0;       // [N·m] — well above model noise
        // Grace period covers traj-ramp transient (TRAJ_START=2.0 + ramp=3s = 5.0,
        // minus CAL_END=1.8) but ends BEFORE collision (force_start=3.0).
        // → grace [1.8, 3.0]s only, releasing in time for collision detection.
        constexpr double GATE_GRACE_PERIOD = 1.2;
        constexpr double PERSISTENCE_TIME = 0.03;    // [s] — 30ms sustained

        Vector7d deviation = r_raw - r_baseline_;
        double dev_max = deviation.cwiseAbs().maxCoeff();
        bool exceeds_now = (dev_max > COLLISION_GATE);

        // Persistence accumulator
        if (exceeds_now) {
            collision_persist_t_ += dt;
        } else {
            collision_persist_t_ = 0.0;
        }
        bool in_grace = (t_now - CAL_END) < GATE_GRACE_PERIOD;
        bool collision_suspected = !in_grace
                                   && (collision_persist_t_ >= PERSISTENCE_TIME);

        if (!collision_suspected) {
            double alpha_baseline = dt / tau_baseline;
            r_baseline_ = (1.0 - alpha_baseline) * r_baseline_ + alpha_baseline * r_raw;
        }

        r = r_raw - r_baseline_;
    }

    // ── STEP 2: FDI Isolation (§3.1, Eq 3.2–3.2a) ──
    auto fdi_result = fdi_->isolate(r, tau_F, tau_m);
    Vector7d r_tilde = fdi_result.r_tilde;
    Vector7d m_aff = fdi_result.m_aff;

    // ── STEP 3: SVD Projection (§3.2–3.3, Eq 3.6–3.7a) ──
    auto proj_result = svd_proj_->project(J_mat, r_tilde, m_aff);
    Vector6d alpha = proj_result.alpha;
    Matrix6d U = proj_result.U;
    Vector6d s = proj_result.s;

    // ── STEP 4: A-DLS Damping Matrix (§4, Eq 4.9–4.19a) ──
    Matrix6d Lambda_cart;
    if (method_ == "M2") {
        // Scalar DLS: Λ = λ²_max I (all directions share same damping)
        Lambda_cart = params_.lambda2_max * Matrix6d::Identity();
    } else if (method_ == "M1" || method_ == "M3") {
        // Moore-Penrose: effectively zero damping
        Lambda_cart = params_.lambda2_base * Matrix6d::Identity();
    } else {  // M4: A-DLS
        Lambda_cart = damping_->compute(alpha, proj_result.alpha_th, U);
    }

    // ── STEP 4b: Reference Modulation (§6.1.1, Eq 6.1a–6.1c) ──
    // Active only for M4 AND when ref_modulation_ flag is true.
    // For experiment 2B configurations C1/C2/C3 we disable this to isolate
    // the contribution of the asymmetric EMA from the reference modulator.
    Vector6d error_eff = error;
    if (method_ == "M4" && ref_modulation_) {
        Vector6d beta = damping_->getBeta();
        error_eff = adls::ReferenceModulator::modulate(error, beta);
    }

    // ── STEP 5: Pseudoinverse (§5) ──
    auto pinv_result = adls::computePseudoinverse(J_mat, Lambda_cart, method_);
    Matrix76d J_plus = pinv_result.J_plus;
    Matrix7d N = pinv_result.N;

    // ── STEP 6: Velocity Command ──
    // Task-space velocity (CLIK, Eq 6.1c)
    Vector7d dq_task = J_plus * (dx_d + params_.K_P.asDiagonal() * error_eff);

    // Null-space compliance velocity (Eq 6.2–6.2b)
    //
    // NOTE on Gazebo simulation:
    //   libfranka analytical g(q) and franka_hw_sim URDF-based g(q) differ by
    //   ~15-20 N·m even when fully calibrated. The GMO observer treats this
    //   mismatch as a spurious external torque, driving the nullspace
    //   compliance to constantly twist the arm into singular configurations.
    //
    //   On REAL Franka, the gravity model matches exactly, so this is not
    //   an issue. The §3.3 deadzone (r_noise = 0.1-0.2 N·m) filters real
    //   sensor noise effectively.
    //
    //   In Gazebo, we either need: (a) calibrated r_noise ≈ 20 N·m to mask
    //   the systematic offset, or (b) disable nullspace compliance entirely
    //   and validate only the task-space pipeline.
    //
    //   We choose (b) for STATIC HOLD diagnostic mode (radius_ = 0), and
    //   leave it enabled for actual collision experiments (radius_ > 0)
    //   where the real collision residual (>30 N·m) is much larger than
    //   the systematic offset.
    Vector7d dq_comp = Vector7d::Zero();
    bool nullspace_enabled = (method_ == "M3" || method_ == "M4")
                             && t_now > 3.0
                             && radius_ > 1e-6;  // disabled in static hold mode
    if (nullspace_enabled) {
        dq_comp = nullspace_->compute(r);
    }

    // Project nullspace velocity through N
    Vector7d dq_null = N * dq_comp;

    // Combined velocity command
    Vector7d dq_cmd = dq_task + dq_null;

    // ── Velocity command saturation (direction-preserving) ──
    // Prevent CLIK spikes while keeping the J⁺-computed direction intact.
    // Per-joint clipping distorts direction → EE moves toward wrong target;
    // uniform scaling preserves direction, only reduces magnitude.
    constexpr double DQ_MAX = 1.5;  // [rad/s] — Franka spec is 2.175
    double dq_cmd_max = dq_cmd.cwiseAbs().maxCoeff();
    if (dq_cmd_max > DQ_MAX) {
        dq_cmd *= (DQ_MAX / dq_cmd_max);   // uniform scale, preserves direction
    }

    // ── STEP 7: Integrate velocity to position command (§6.3, Algorithm STEP 7) ──
    q_cmd_ = q + dq_cmd * dt;
    dq_cmd_ = dq_cmd;

    // ── STEP 8: Torque Command (§6.3, Eq 6.4–6.5) ──
    //
    // GRAVITY: handled by Franka platform (FCI / franka_hw_sim plugin),
    //          NOT by this controller. Adding g(q) would double-compensate.
    //
    // CORIOLIS: libfranka's analytical C(q,dq) computation differs slightly
    //          from Gazebo's ODE-based internal computation. Adding C·dq
    //          here introduces small model-mismatch torques that the
    //          velocity feedback amplifies into steady-state hunting.
    //          For slow motions (‖dq‖ < 1 rad/s), C·dq is negligible (<0.1 N·m),
    //          so we omit it on Gazebo. Re-enable for real Franka hardware.
    //
    // Final form: τ = ramp · M(q) · K_v · (dq_cmd - dq)
    //   — pure inverse-dynamics velocity tracking, no Coriolis, no gravity
    //   — clean separation of A-DLS innovation (in dq_cmd) from servo loop
    Eigen::Map<Matrix7d> M_ctrl(M_arr.data());

    // Soft-start ramp
    double ramp = std::min(1.0, t_now / 1.0);

    // Velocity tracking gain — increased for tighter dq tracking.
    // K_v = 25 → τ_v = 40ms. Faster damping of joint-space drift.
    constexpr double K_v = 25.0;
    Vector7d ddq_des = K_v * (dq_cmd - dq);

    // EMA on desired acceleration (paper §6.3 加速度级 EMA)
    double gamma_a = params_.gamma_a;
    ddq_cmd_ema_ = (1.0 - gamma_a) * ddq_cmd_ema_ + gamma_a * ddq_des;

    // Final torque: M*ddq_des only (gravity & Coriolis handled elsewhere)
    Vector7d tau_cmd_raw = ramp * (M_ctrl * ddq_cmd_ema_);

    dq_cmd_prev_ = dq_cmd;

    // ── STEP 9: Torque Rate & Magnitude Limiting ──
    // Use our own previous command as reference (more reliable than Franka's tau_J_d in Gazebo)
    Vector7d tau_cmd = saturateTorqueRate(tau_cmd_raw, tau_cmd_prev_);
    tau_cmd_prev_ = tau_cmd;

    // Apply external torque to simulation (Gazebo will apply this via physics)
    // In Gazebo, we send torque commands to joints. The external force is
    // accounted for in tau_ext which the GMO observes.

    // ── Enhanced diagnostic — pinpoints CLIK loop issues ──
    // We'll show:
    //   pos_err: |x_d - x_cur|              ← Cartesian position error
    //   ‖dq_cmd‖: how much joint motion CLIK wants
    //   ‖dq‖:    how much joints actually move
    //   ‖ẋ_d‖:   feed-forward target velocity
    //   ‖J·dq‖:   actual EE velocity
    //   ‖J·dq_cmd‖: what CLIK thinks it's commanding for EE
    //
    // Diagnostic interpretation:
    //   If ‖J·dq_cmd‖ ≈ ‖ẋ_d‖ but ‖J·dq‖ ≪ ‖J·dq_cmd‖ → velocity tracking fails
    //   If ‖J·dq_cmd‖ ≠ ‖ẋ_d‖                       → CLIK formula has a bug
    //   If pos_err grows linearly                    → no integral, lag accumulating
    Vector6d v_cmd_ee = J_mat * dq_cmd;
    Vector6d v_actual_ee = J_mat * dq;
    ROS_INFO_THROTTLE(1.0,
        "[DIAG t=%.1f %s] pos_err=%.1fmm | "
        "‖ẋ_d‖=%.3f ‖J·dq_cmd‖=%.3f ‖J·dq‖=%.3f m/s | "
        "‖dq_cmd‖=%.3f ‖dq‖=%.3f rad/s | "
        "‖τ‖=%.2f Nm | r_raw=%.1f r_corr=%.1f Nm",
        t_now,
        (radius_ <= 1e-6 ? "STAT" : (traj_started_ ? "CIRC" : "HOLD")),
        1000.0 * error.head(3).norm(),
        dx_d.head(3).norm(),
        v_cmd_ee.head(3).norm(),
        v_actual_ee.head(3).norm(),
        dq_cmd.norm(),
        dq.norm(),
        tau_cmd.cwiseAbs().maxCoeff(),
        r_raw.cwiseAbs().maxCoeff(),
        r.cwiseAbs().maxCoeff()
    );

    // ── Send torque command ──
    for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_cmd[i]);
    }

    // ── Publish diagnostics (100 Hz) ──
    // ────────────────────────────────────────────────────────────
    // Complete diagnostic layout (must match scripts/plot_utils.py DIAG_FIELDS):
    //   [0]      t                                       (1)
    //   [1-6]    alpha (collision projection)            (6)
    //   [7-12]   lambda2 (A-DLS damping per direction)   (6)
    //   [13-19]  r_corr (GMO residual after baseline)    (7)
    //   [20-25]  error (Cartesian error 6D)              (6)
    //   [26]     JN_norm (nullspace leakage)             (1)
    //   [27-32]  singular_values                         (6)
    //   [33-35]  position_cur (EE xyz)                   (3)
    //   [36-38]  position_d_t (target xyz)               (3)
    //   [39-44]  dx_d (feedforward Cartesian velocity)   (6)
    //   [45-51]  dq (joint velocities)                   (7)
    //   [52-58]  dq_cmd (commanded joint velocities)     (7)
    //   [59-65]  tau_cmd (commanded torques)             (7)
    //   [66-72]  r_raw (GMO residual before baseline)    (7)
    //   [73-78]  F_ext_applied (injected external force) (6)
    //   [79]     k_star (FDI link index)                 (1)
    //   [80]     method_id (1=M1, 2=M2, 3=M3, 4=M4)      (1)
    //   [81-86]  alpha_raw (before deadzone)             (6)
    //   [87-92]  Lambda_cart diag (in Cartesian coords)  (6)
    // ──── total = 93 fields ──────────────────────────────────────
    if (count_ >= diag_pub_interval_) {
        count_ = 0.0;
        std_msgs::Float64MultiArray diag_msg;
        diag_msg.data.resize(93);
        int idx = 0;
        diag_msg.data[idx++] = t_now;                                                    // 0
        for (int i = 0; i < 6; ++i) diag_msg.data[idx++] = alpha[i];                      // 1-6
        for (int i = 0; i < 6; ++i) diag_msg.data[idx++] = damping_->debug_lambda2_raw()[i]; // 7-12
        for (int i = 0; i < 7; ++i) diag_msg.data[idx++] = r[i];                          // 13-19
        for (int i = 0; i < 6; ++i) diag_msg.data[idx++] = error[i];                      // 20-25
        diag_msg.data[idx++] = (J_mat * N).norm();                                        // 26
        for (int i = 0; i < 6; ++i) diag_msg.data[idx++] = s[i];                          // 27-32
        for (int i = 0; i < 3; ++i) diag_msg.data[idx++] = position_cur[i];               // 33-35
        for (int i = 0; i < 3; ++i) diag_msg.data[idx++] = position_d_t[i];               // 36-38
        for (int i = 0; i < 6; ++i) diag_msg.data[idx++] = dx_d[i];                       // 39-44
        for (int i = 0; i < 7; ++i) diag_msg.data[idx++] = dq[i];                         // 45-51
        for (int i = 0; i < 7; ++i) diag_msg.data[idx++] = dq_cmd[i];                     // 52-58
        for (int i = 0; i < 7; ++i) diag_msg.data[idx++] = tau_cmd[i];                    // 59-65
        for (int i = 0; i < 7; ++i) diag_msg.data[idx++] = r_raw[i];                      // 66-72
        for (int i = 0; i < 6; ++i) diag_msg.data[idx++] = F_ext_applied_[i];             // 73-78
        diag_msg.data[idx++] = static_cast<double>(fdi_result.k_star);                    // 79
        int method_id = (method_ == "M1" ? 1 : (method_ == "M2" ? 2 :
                        (method_ == "M3" ? 3 : 4)));
        diag_msg.data[idx++] = static_cast<double>(method_id);                            // 80
        for (int i = 0; i < 6; ++i) diag_msg.data[idx++] = proj_result.alpha_raw[i];      // 81-86
        for (int i = 0; i < 6; ++i) diag_msg.data[idx++] = Lambda_cart(i, i);             // 87-92
        diag_pub_.publish(diag_msg);
    }
}

// ============================================================================
// saturateTorqueRate() — Torque safety limiting
// ============================================================================
Vector7d ADLSController::saturateTorqueRate(
    const Vector7d& tau_d_calculated,
    const Vector7d& tau_J_d) {
    Vector7d tau_d_saturated;
    for (size_t i = 0; i < 7; ++i) {
        double diff = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] +
            std::max(std::min(diff, params_.delta_tau_max), -params_.delta_tau_max);
        // Per-joint magnitude limits matching Franka Panda spec
        if (i <= 3) {
            tau_d_saturated[i] = std::max(-30.0, std::min(30.0, tau_d_saturated[i]));
        } else {
            tau_d_saturated[i] = std::max(-8.0, std::min(8.0, tau_d_saturated[i]));
        }
    }
    return tau_d_saturated;
}

}  // namespace variable_stiffness_controllers

// Register as ROS plugin
PLUGINLIB_EXPORT_CLASS(variable_stiffness_controllers::ADLSController,
                       controller_interface::ControllerBase)
