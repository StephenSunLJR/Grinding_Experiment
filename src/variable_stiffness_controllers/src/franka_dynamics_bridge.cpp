/*
 * franka_dynamics_bridge.cpp
 *
 * C-compatible wrapper for Franka Panda dynamics functions.
 * Exposes MassMatrix, CoriolisMatrix, GravityVector, Friction
 * as extern "C" functions with plain double arrays for Python ctypes interop.
 *
 * Compile:
 *   g++ -shared -fPIC -O2 -o libfranka_dynamics_bridge.so franka_dynamics_bridge.cpp \
 *       MassMatrix.cpp CoriolisMatrix.cpp GravityVector.cpp FrictionTorque.cpp \
 *       -I../include
 *
 * Usage from Python:
 *   import ctypes
 *   lib = ctypes.CDLL('./libfranka_dynamics_bridge.so')
 *   lib.mass_matrix(q_ptr, M_ptr)  # q: double[7], M: double[49] (column-major)
 */

#include "../include/franka_model.h"

extern "C" {

// --- Mass Matrix M(q): 7×7 → 49 doubles (column-major, Eigen default) ---
void franka_mass_matrix(const double* q, double* M_out) {
    Eigen::Map<const Eigen::Vector7d> q_vec(q);
    Eigen::Map<Eigen::Matrix7d> M(M_out);
    M = MassMatrix(q_vec);
}

// --- Coriolis Matrix C(q,dq): 7×7 → 49 doubles ---
void franka_coriolis_matrix(const double* q, const double* dq, double* C_out) {
    Eigen::Map<const Eigen::Vector7d> q_vec(q);
    Eigen::Map<const Eigen::Vector7d> dq_vec(dq);
    Eigen::Map<Eigen::Matrix7d> C(C_out);
    C = CoriolisMatrix(q_vec, dq_vec);
}

// --- Coriolis+Centrifugal torque C(q,dq)*dq: 7 → 7 doubles ---
void franka_coriolis_torque(const double* q, const double* dq, double* c_out) {
    Eigen::Map<const Eigen::Vector7d> q_vec(q);
    Eigen::Map<const Eigen::Vector7d> dq_vec(dq);
    Eigen::Map<Eigen::Vector7d> c(c_out);
    c = CoriolisMatrix(q_vec, dq_vec) * dq_vec;
}

// --- Gravity vector g(q): 7 → 7 doubles ---
void franka_gravity_vector(const double* q, double* g_out) {
    Eigen::Map<const Eigen::Vector7d> q_vec(q);
    Eigen::Map<Eigen::Vector7d> g(g_out);
    g = GravityVector(q_vec);
}

// --- Friction torque tau_F(dq): 7 → 7 doubles ---
void franka_friction_torque(const double* dq, double* tau_f_out) {
    Eigen::Map<const Eigen::Vector7d> dq_vec(dq);
    Eigen::Map<Eigen::Vector7d> tau_f(tau_f_out);
    tau_f = Friction(dq_vec);
}

}  // extern "C"
