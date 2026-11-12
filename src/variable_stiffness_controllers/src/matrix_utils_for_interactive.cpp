#include "../include/matrix_utils_for_interactive.h"

namespace interactive_force_impedence_control { 
    void calculate_universal_impedence_matrices(
        const Matrix3d& R_W_F,
        const Vector6d& Fd_bin,
        Matrix6d& D_i,
        Matrix6d& D_i_c,
        Matrix6d& D_i_u,
        Matrix6d& R6_W_F
    ){
        //Matrix6d  R6_W_F = Matrix6d::Zero();
        R6_W_F.block<3,3>(0,0) = R_W_F;
        R6_W_F.block<3,3>(3,3) = R_W_F;
        D_i = R6_W_F * Fd_bin.asDiagonal() ;
        Matrix6d DiT_Di = D_i.transpose() * D_i;
        Eigen::MatrixXd DiT_Di_pinv;

        franka_interactive_controllers::pseudoInverse(DiT_Di, DiT_Di_pinv);
        D_i_c = D_i * DiT_Di_pinv * D_i.transpose();

        D_i_u = Matrix6d::Identity() - D_i_c;

    }
}