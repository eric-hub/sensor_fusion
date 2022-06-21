/*
 * @Description: ceres residual block for LIO IMU pre-integration measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGIMUPreIntegration : public ceres::SizedCostFunction<15, 15, 15> {
public:
    static const int INDEX_P = 0;
    static const int INDEX_R = 3;
    static const int INDEX_V = 6;
    static const int INDEX_A = 9;
    static const int INDEX_G = 12;

    FactorPRVAGIMUPreIntegration(void){};

    void SetT(const double &T) {
        T_ = T;
    }

    void SetGravitiy(const Eigen::Vector3d &g) {
        g_ = g;
    }

    void SetMeasurement(const Eigen::VectorXd &m) {
        m_ = m;
    }

    void SetInformation(const Eigen::MatrixXd &I) {
        I_ = I;
    }

    void SetJacobian(const Eigen::MatrixXd &J) {
        J_ = J;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        //
        // parse parameters:
        //
        // a. pose i
        Eigen::Map<const Eigen::Vector3d> pos_i(&parameters[0][INDEX_P]);
        Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
        const Sophus::SO3d ori_i = Sophus::SO3d::exp(log_ori_i);
        Eigen::Map<const Eigen::Vector3d> vel_i(&parameters[0][INDEX_V]);
        Eigen::Map<const Eigen::Vector3d> b_a_i(&parameters[0][INDEX_A]);
        Eigen::Map<const Eigen::Vector3d> b_g_i(&parameters[0][INDEX_G]);

        // b. pose j
        Eigen::Map<const Eigen::Vector3d> pos_j(&parameters[1][INDEX_P]);
        Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
        const Sophus::SO3d ori_j = Sophus::SO3d::exp(log_ori_j);
        Eigen::Map<const Eigen::Vector3d> vel_j(&parameters[1][INDEX_V]);
        Eigen::Map<const Eigen::Vector3d> b_a_j(&parameters[1][INDEX_A]);
        Eigen::Map<const Eigen::Vector3d> b_g_j(&parameters[1][INDEX_G]);

        //
        // parse measurement:
        //
        Eigen::Vector3d alpha_ij = m_.block<3, 1>(INDEX_P, 0);
        Eigen::Vector3d theta_ij = m_.block<3, 1>(INDEX_R, 0);
        Eigen::Vector3d beta_ij = m_.block<3, 1>(INDEX_V, 0);

        Eigen::Vector3d dba = b_a_i - m_.block<3, 1>(INDEX_A, 0);
        Eigen::Vector3d dbg = b_g_i - m_.block<3, 1>(INDEX_G, 0);

        Eigen::Matrix3d J_q_bg = J_.block<3, 3>(INDEX_R, INDEX_G);
        Eigen::Matrix3d J_v_ba = J_.block<3, 3>(INDEX_V, INDEX_A);
        Eigen::Matrix3d J_v_bg = J_.block<3, 3>(INDEX_V, INDEX_G);
        Eigen::Matrix3d J_p_ba = J_.block<3, 3>(INDEX_P, INDEX_A);
        Eigen::Matrix3d J_p_bg = J_.block<3, 3>(INDEX_P, INDEX_G);

        alpha_ij = alpha_ij + J_p_ba * dba + J_p_bg * dbg;
        Sophus::SO3d ori_ij = Sophus::SO3d::exp(theta_ij) * Sophus::SO3d::exp(J_q_bg * dbg);
        beta_ij = beta_ij + J_v_ba * dba + J_v_bg * dbg;

        //
        // TODO: get square root of information matrix:
        //

        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(I_).matrixL().transpose();

        //
        // TODO: compute residual:
        //
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);

        residual.block<3, 1>(INDEX_P, 0) = ori_i.inverse().matrix() * (pos_j - pos_i - (vel_i - 0.5 * g_ * T_) * T_) - alpha_ij;
        residual.block<3, 1>(INDEX_R, 0) = (ori_ij.inverse() * ori_i.inverse() * ori_j).log();
        residual.block<3, 1>(INDEX_V, 0) = ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
        residual.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
        residual.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;

        //
        // TODO: compute jacobians:
        //
        if (jacobians) {
            // compute shared intermediate results:
            const Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();
            const Eigen::Matrix3d J_r_inv = JacobianRInv(residual.block(INDEX_R, 0, 3, 1)); //  右雅克比

            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_i(jacobians[0]);
                jacobian_i.setZero();

                // a. residual, position:
                jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -R_i_inv;
                // Eigen::Vector3d one(1, 1, 1);
                // jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -one;
                jacobian_i.block<3, 3>(INDEX_P, INDEX_R) =
                    Sophus::SO3d::hat(
                        ori_i.inverse() * (pos_j - pos_i - vel_i * T_ + 0.50 * g_ * T_ * T_));
                jacobian_i.block<3, 3>(INDEX_P, INDEX_V) = -T_ * R_i_inv;
                jacobian_i.block<3, 3>(INDEX_P, INDEX_A) = -J_.block<3, 3>(INDEX_P, INDEX_A);
                jacobian_i.block<3, 3>(INDEX_P, INDEX_G) = -J_.block<3, 3>(INDEX_P, INDEX_G);

                // b. residual, orientation:
                jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -J_r_inv * (ori_j.inverse() * ori_i).matrix();
                jacobian_i.block<3, 3>(INDEX_R, INDEX_G) =
                    -J_r_inv *
                    (Sophus::SO3d::exp(residual.block<3, 1>(INDEX_R, 0))).matrix().inverse() *
                    JacobianR(J_.block<3, 3>(INDEX_R, INDEX_G) * dbg) *
                    J_.block<3, 3>(INDEX_R, INDEX_G);

                // c. residual, velocity:
                jacobian_i.block<3, 3>(INDEX_V, INDEX_R) =
                    Sophus::SO3d::hat(
                        ori_i.inverse() * (vel_j - vel_i + g_ * T_));
                jacobian_i.block<3, 3>(INDEX_V, INDEX_V) = -R_i_inv;
                jacobian_i.block<3, 3>(INDEX_V, INDEX_A) = -J_.block<3, 3>(INDEX_V, INDEX_A);
                jacobian_i.block<3, 3>(INDEX_V, INDEX_G) = -J_.block<3, 3>(INDEX_V, INDEX_G);

                // d. residual, bias accel:
                jacobian_i.block<3, 3>(INDEX_A, INDEX_A) = -Eigen::Matrix3d::Identity();

                // d. residual, bias accel:
                jacobian_i.block<3, 3>(INDEX_G, INDEX_G) = -Eigen::Matrix3d::Identity();

                jacobian_i = sqrt_info * jacobian_i;
            }

            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_j(jacobians[1]);
                jacobian_j.setZero();

                // a. residual, position:
                jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = R_i_inv;

                // b. residual, orientation:
                jacobian_j.block<3, 3>(INDEX_R, INDEX_R) = J_r_inv;

                // c. residual, velocity:
                jacobian_j.block<3, 3>(INDEX_V, INDEX_V) = R_i_inv;

                // d. residual, bias accel:
                jacobian_j.block<3, 3>(INDEX_A, INDEX_A) = Eigen::Matrix3d::Identity();

                // d. residual, bias accel:
                jacobian_j.block<3, 3>(INDEX_G, INDEX_G) = Eigen::Matrix3d::Identity();

                jacobian_j = sqrt_info * jacobian_j;
            }
        }

        //
        // TODO: correct residual by square root of information matrix:
        //

        residual = sqrt_info * residual;

        return true;
    }

private:
    static Eigen::Matrix3d JacobianR(const Eigen::Vector3d &w) {
        Eigen::Matrix3d J_r = Eigen::Matrix3d::Identity();

        double theta = w.norm();

        if (theta > 1e-5) {
            Eigen::Vector3d a = w.normalized();
            Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
            double theta_sin = sin(theta);
            double theta_cos = cos(theta);

            J_r = (theta_sin / theta) * J_r + (1 - theta_sin / theta) * a * a.transpose() - ((1 - theta_cos) / theta) * a_hat;
        }

        return J_r;
    }

    static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
        Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

        double theta = w.norm();

        if (theta > 1e-5) {
            Eigen::Vector3d a = w.normalized();
            Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
            double theta_half = 0.5 * theta;
            double cot_theta = 1.0 / tan(theta_half);

            J_r_inv = theta_half * cot_theta * J_r_inv + (1.0 - theta_half * cot_theta) * a * a.transpose() +
                      theta_half * a_hat;
        }

        return J_r_inv;
    }

    double T_ = 0.0;

    Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

    Eigen::VectorXd m_;
    Eigen::MatrixXd I_;

    Eigen::MatrixXd J_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
