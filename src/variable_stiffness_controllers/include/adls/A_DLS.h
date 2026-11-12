#pragma once
/**
 * @file A_DLS.h
 * @brief Anisotropic DLS Controller — ROS controller plugin for Franka Panda.
 *
 * Implements the complete A-DLS pipeline (Algorithm 1 of the paper):
 *   STEP 1: GMO Observer → r                  (§2, Eq 2.9)
 *   STEP 2: FDI Isolator → r̃, m_aff, k*       (§3.1, Eq 3.2–3.2a)
 *   STEP 3: SVD Projector → α, U, s, V₁       (§3.2–3.3, Eq 3.6–3.7a)
 *   STEP 4: A-DLS Damping → Λ_cart            (§4, Eq 4.9–4.19a)
 *   STEP 4b: Reference Modulator → x̃_eff       (§6.1.1, Eq 6.1a–6.1c)
 *   STEP 5: Pseudoinverse → J⁺, N             (M1/M2/M3/M4)
 *   STEP 6: Task + Nullspace velocity → q̇_cmd
 *   STEP 7: CTC Torque → τ_cmd                (§6.3, Eq 6.4–6.5)
 *
 * Supports 4 comparison methods:
 *   M1: Moore-Penrose (strict priority)
 *   M2: Scalar DLS
 *   M3: Moore-Penrose + nullspace compliance
 *   M4: A-DLS (proposed, default)
 */

#include <memory>
#include <string>
#include <vector>
#include <array>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// Sub-modules
#include "types.h"
#include "gmo_observer.h"
#include "fdi_isolator.h"
#include "svd_projector.h"
#include "adls_damping.h"
#include "nullspace_compliance.h"
#include "reference_modulator.h"
#include "pseudo_inverse_mux.h"

namespace variable_stiffness_controllers {

class ADLSController : public controller_interface::MultiInterfaceController<
                           franka_hw::FrankaModelInterface,
                           hardware_interface::EffortJointInterface,
                           franka_hw::FrankaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw,
              ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

private:
    // ── Torque rate & magnitude safety ──
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);

    // ── Forward kinematics using libfranka ──
    Eigen::Matrix<double, 6, 1> getEndEffectorPose(
        const std::array<double, 16>& O_T_EE);

    // ── ROS interfaces ──
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    // ── A-DLS modules (initialized in init() after params loaded) ──
    adls::ADLSParams params_;
    std::string method_{"M4"};

    // ── Experiment 2B (slingshot ablation) options ──
    // ema_mode: "none" (raw), "symmetric" (paper v5.9 baseline), "asymmetric" (v5.10)
    // ref_modulation: enable §6.1.1 reference-trajectory modulation
    std::string ema_mode_{"asymmetric"};
    bool ref_modulation_{true};

    // ── Trajectory mode ──
    // "circle": default XY circle (experiment 1, 2A, 2B)
    // "extend": singularity-test trajectory (experiment 3B)
    std::string traj_mode_{"circle"};
    double extend_distance_{0.30};   // [m] +X extension for singular test
    std::unique_ptr<adls::GMOObserver> gmo_;
    std::unique_ptr<adls::FDIIsolator> fdi_;
    std::unique_ptr<adls::SVDProjector> svd_proj_;
    std::unique_ptr<adls::ADLSDamping> damping_;
    std::unique_ptr<adls::NullSpaceCompliance> nullspace_;

    // ── CTC state ──
    Eigen::Matrix<double, 7, 1> q_cmd_{Eigen::Matrix<double, 7, 1>::Zero()};
    Eigen::Matrix<double, 7, 1> dq_cmd_{Eigen::Matrix<double, 7, 1>::Zero()};
    Eigen::Matrix<double, 7, 1> dq_cmd_prev_{Eigen::Matrix<double, 7, 1>::Zero()};
    Eigen::Matrix<double, 7, 1> ddq_cmd_ema_{Eigen::Matrix<double, 7, 1>::Zero()};
    Eigen::Matrix<double, 7, 1> tau_cmd_prev_{Eigen::Matrix<double, 7, 1>::Zero()};
    bool ctc_initialized_{false};

    // ── Experiment / trajectory state ──
    ros::Duration t_{0.0};
    double count_{0.0};
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    double init_roll_{0.0}, init_pitch_{0.0}, init_yaw_{0.0};
    Eigen::Matrix<double, 6, 1> velocity_d_{Eigen::Matrix<double, 6, 1>::Zero()};
    double radius_{0.05};
    double factor_circle_{2.0 * M_PI / 5.0};  // 5s period
    bool traj_started_{false};

    // ── GMO baseline calibration (compensates Gazebo's systematic offset) ──
    Eigen::Matrix<double, 7, 1> r_baseline_{Eigen::Matrix<double, 7, 1>::Zero()};
    int baseline_samples_{0};
    bool baseline_calibrated_{false};
    double collision_persist_t_{0.0};   // persistence accumulator for baseline-freeze gate

    // ── External force parameters (experiment manager) ──
    double force_start_time_{3.0};
    double force_end_time_{5.0};
    double force_ramp_time_{0.2};
    Eigen::Matrix<double, 6, 1> F_ext_applied_{Eigen::Matrix<double, 6, 1>::Zero()};
    double force_magnitude_{25.0};   // [N]
    int force_direction_{2};         // 0=X, 1=Y, 2=Z

    // ── Diagnostics publisher ──
    ros::Publisher diag_pub_;
    double diag_pub_interval_{0.01};  // 100 Hz
    double diag_last_pub_time_{0.0};
};

}  // namespace variable_stiffness_controllers
