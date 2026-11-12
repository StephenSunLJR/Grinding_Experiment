#pragma once

#include <Eigen/Dense>
#include "franka_utils/pseudo_inversion.h"
#include "interactive_control.h"

typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,3,3> Matrix3d;
typedef Eigen::Matrix<double,6,1> Vector6d;

namespace interactive_force_impedence_control{

    void calculate_universal_impedence_matrices(
        const Matrix3d& R_W_F,
        const Vector6d& Fd_bin,
        Matrix6d& D_i,
        Matrix6d& D_i_c,
        Matrix6d& D_i_u,
        Matrix6d& R6_W_F
    );
}
