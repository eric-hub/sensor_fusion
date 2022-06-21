/*
 * @Description: ceres residual block for map matching pose measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_HPP_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGMapMatchingPose : public ceres::SizedCostFunction<6, 15> {
public:
    static const int INDEX_P = 0;
    static const int INDEX_R = 3;

    FactorPRVAGMapMatchingPose(void){};

    void SetMeasurement(const Eigen::VectorXd &m) {
        m_ = m;
    }

    void SetInformation(const Eigen::MatrixXd &I) {
        I_ = I;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        //
        // parse parameters:
        //
        // pose
        Eigen::Map<const Eigen::Vector3d> pos(&parameters[0][INDEX_P]);
        Eigen::Map<const Eigen::Vector3d> log_ori(&parameters[0][INDEX_R]);
        const Sophus::SO3d ori = Sophus::SO3d::exp(log_ori);

        //
        // parse measurement:
        //
        const Eigen::Vector3d &pos_prior = m_.block<3, 1>(INDEX_P, 0);
        const Eigen::Vector3d &log_ori_prior = m_.block<3, 1>(INDEX_R, 0);
        const Sophus::SO3d ori_prior = Sophus::SO3d::exp(log_ori_prior);

        //
        // TODO: get square root of information matrix:
        //
        Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(I_).matrixL().transpose();
        //
        // TODO: compute residual:
        //

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);

        residual.block(INDEX_P, 0, 3, 1) = pos - pos_prior;
        residual.block(INDEX_R, 0, 3, 1) = (ori * ori_prior.inverse()).log();

        //
        // TODO: compute jacobians:
        //
        if (jacobians) {
            if (jacobians[0]) {
                // implement jacobian computing:
                Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_prior(jacobians[0]);
                jacobian_prior.setZero();

                jacobian_prior.block<3, 3>(INDEX_P, INDEX_P) = Eigen::Matrix3d::Identity();
                jacobian_prior.block<3, 3>(INDEX_R, INDEX_R) =
                    JacobianRInv(
                        residual.block(INDEX_R, 0, 3, 1)) *
                    ori_prior.matrix();

                jacobian_prior = sqrt_info * jacobian_prior;
            }
        }

        //
        // TODO: correct residual by square root of information matrix:
        //

        residual = sqrt_info * residual;

        return true;
    }

private:
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

    Eigen::VectorXd m_;
    Eigen::MatrixXd I_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_HPP_
