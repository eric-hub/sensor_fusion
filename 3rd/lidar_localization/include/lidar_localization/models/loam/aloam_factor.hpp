// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

//
// TODO: implement analytic Jacobians for LOAM residuals in this file
//

#ifndef LIDAR_LOCALIZATION_MODELS_ALOAM_FACTOR_HPP_
#define LIDAR_LOCALIZATION_MODELS_ALOAM_FACTOR_HPP_

#include <eigen3/Eigen/Dense>

//
// TODO: Sophus is ready to use if you have a good undestanding of Lie algebra.
//
#include <sophus/so3.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

inline Eigen::Matrix<double, 3, 3> skew(Eigen::Matrix<double, 3, 1> &mat_in) {
    Eigen::Matrix<double, 3, 3> skew_mat;
    skew_mat.setZero();
    skew_mat(0, 1) = -mat_in(2);
    skew_mat(0, 2) = mat_in(1);
    skew_mat(1, 2) = -mat_in(0);
    skew_mat(1, 0) = mat_in(2);
    skew_mat(2, 0) = -mat_in(1);
    skew_mat(2, 1) = mat_in(0);
    return skew_mat;
}

class LidarEdgeFactor : public ceres::SizedCostFunction<1, 4, 3> { // 优化参数维度：1     输入维度 ： q : 4   t : 3
public:
    double s;
    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    LidarEdgeFactor(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                    const Eigen::Vector3d last_point_b_, const double s_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_,
                                       const double s_) {
        return new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_);
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const //   定义残差模型
    {
        Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]); //   存放 w  x y z
        Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);
        Eigen::Vector3d lp; //   line point
        Eigen::Vector3d lp_r;
        lp_r               = q_last_curr * curr_point;
        lp                 = q_last_curr * curr_point + t_last_curr; //   new point
        Eigen::Vector3d nu = (lp - last_point_b).cross(lp - last_point_a);
        Eigen::Vector3d de = last_point_a - last_point_b;

        residuals[0] = nu.norm() / de.norm(); //  线残差

        //  归一单位化
        // nu.normalize();

        if (jacobians != NULL) {
            if (jacobians[0] != NULL) {
                // Eigen::Vector3d re = last_point_b - last_point_a;
                // Eigen::Matrix3d skew_re = skew(re);
                Eigen::Matrix3d skew_de = skew(de);

                //  J_so3_Rotation
                Eigen::Matrix3d skew_lp_r = skew(lp_r);
                Eigen::Matrix3d dp_by_dr;
                dp_by_dr.block<3, 3>(0, 0) = -skew_lp_r;
                Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
                J_so3_r.setZero();
                J_so3_r.block<1, 3>(0, 0) = nu.transpose() * skew_de * dp_by_dr / (residuals[0]);

                //  J_so3_Translation
                Eigen::Matrix3d dp_by_dt;
                (dp_by_dt.block<3, 3>(0, 0)).setIdentity();
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_so3_t(jacobians[1]);
                J_so3_t.setZero();
                J_so3_t.block<1, 3>(0, 0) = nu.transpose() * skew_de / (residuals[0]);
            }
        }
        return true;
    }
};

class LidarPlaneFactor : public ceres::SizedCostFunction<1, 4, 3> {
public:
    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
    double s;

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d last_point_j_,
                                       const Eigen::Vector3d last_point_l_,
                                       const Eigen::Vector3d last_point_m_,
                                       const double s_) {
        // return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
        // new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_,
        // last_point_m_, s_)));

        return new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_,
                                    last_point_m_, s_);
    }

    LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                     Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
        : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_), last_point_m(last_point_m_), s(s_) {}

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const { //   定义残差模型
        // 叉乘运算， j,l,m 三个但构成的平行四边面积(摸)和该面的单位法向量(方向)
        Eigen::Vector3d ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
        ljm_norm.normalize(); //  单位法向量

        Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);

        Eigen::Vector3d lp;                              // “从当前阵的当前点” 经过转换矩阵转换到“上一阵的同线束激光点”
        Eigen::Vector3d lp_r = q_last_curr * curr_point; //  for compute jacobian o rotation  L: dp_dr
        lp                   = q_last_curr * curr_point + t_last_curr;

        // 残差函数
        double phi1  = (lp - last_point_j).dot(ljm_norm);
        residuals[0] = std::fabs(phi1);

        if (jacobians != NULL) {
            if (jacobians[0] != NULL) {
                phi1 = phi1 / residuals[0];
                //  Rotation
                Eigen::Matrix3d skew_lp_r = skew(lp_r);
                Eigen::Matrix3d dp_dr;
                dp_dr.block<3, 3>(0, 0) = -skew_lp_r;
                Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
                J_so3_r.setZero();
                J_so3_r.block<1, 3>(0, 0) = phi1 * ljm_norm.transpose() * (dp_dr);

                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_so3_t(jacobians[1]);
                J_so3_t.block<1, 3>(0, 0) = phi1 * ljm_norm.transpose();
            }
        }
        return true;
    }
};

struct LidarPlaneNormFactor {
    LidarPlaneNormFactor(Eigen::Vector3d curr_point_,
                         Eigen::Vector3d plane_unit_norm_,
                         double negative_OA_dot_norm_)
        : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_) {}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                                  T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_w_curr * cp + t_w_curr;

        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()),
                                    T(plane_unit_norm.z()));
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_) {
        return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 4, 3>(
            new LidarPlaneNormFactor(curr_point_, plane_unit_norm_,
                                     negative_OA_dot_norm_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
};

struct LidarDistanceFactor {
    LidarDistanceFactor(Eigen::Vector3d curr_point_,
                        Eigen::Vector3d closed_point_)
        : curr_point(curr_point_), closed_point(closed_point_) {}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                                  T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_w_curr * cp + t_w_curr;

        residual[0] = point_w.x() - T(closed_point.x());
        residual[1] = point_w.y() - T(closed_point.y());
        residual[2] = point_w.z() - T(closed_point.z());
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d closed_point_) {
        return (new ceres::AutoDiffCostFunction<LidarDistanceFactor, 3, 4, 3>(
            new LidarDistanceFactor(curr_point_, closed_point_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d closed_point;
};

#endif
