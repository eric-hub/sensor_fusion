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

class LidarEdgeFactor : public ceres::SizedCostFunction<1, 4, 3> {
  LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                  Eigen::Vector3d last_point_b_, double s_)
      : curr_point(curr_point_), last_point_a(last_point_a_),
        last_point_b(last_point_b_), s(s_) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    // Eigen::Quaterniond q_identity(1, 0, 0, 0);
    // q_last_curr = q_identity.slerp(s, q_last_curr);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);
    Eigen::Vector3d lp = q_last_curr * curr_point + t_last_curr; //   new point

    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
    Eigen::Vector3d de = last_point_a - last_point_b;
    residuals[0] = nu.norm() / de.norm();

    Eigen::Matrix3d J_xi_p = skew(de) / de.norm();
    Eigen::Vector4d q = q_last_curr.coeffs();

    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Matrix<double, 3, 4> J_x_q;
        double qx = q(0), qy = q(1), qz = q(2), qw = q(3);
        double X = curr_point(0), Y = curr_point(1), Z = curr_point(2);
        // https://www.cnblogs.com/JingeTU/p/11707557.html
        J_x_q << qy * Y + qz * Z, qw * Z + qx * Y - 2 * qy * X,
            -qw * Y + qx * Z - 2 * qz * X, qy * Z - qz * Y,
            -qw * Z - 2 * qx * Y + qy * X, qx * X + qz * Z,
            qw * X + qy * Z - 2 * qz * Y, -qx * Z + qz * X,
            qw * Y - 2 * qx * Z + qz * X, -qw * X - 2 * qy * Z + qz * Y,
            qx * X + qy * Y, qx * Y - qy * X;
        // https://zhuanlan.zhihu.com/p/131342530
        // J_x_q << 2 * X * qx + 2 * Y * qy + 2 * Z * qz,
        //     -2 * X * qy + 2 * Y * qx + 2 * Z * qw,
        //     -2 * X * qz - 2 * Y * qw + 2 * Z * qx,
        //     2 * X * qw - 2 * Y * qz + 2 * Z * qy,
        //     2 * X * qy - 2 * Y * qx - 2 * Z * qw,
        //     2 * X * qx + 2 * Y * qy + 2 * Z * qz,
        //     2 * X * qw - 2 * Y * qz + 2 * Z * qy,
        //     2 * X * qz + 2 * Y * qw - 2 * Z * qx,
        //     2 * X * qz + 2 * Y * qw - 2 * Z * qx,
        //     -2 * X * qw + 2 * Y * qz - 2 * Z * qy,
        //     2 * X * qx + 2 * Y * qy + 2 * Z * qz,
        //     -2 * X * qy + 2 * Y * qx + 2 * Z * qw;
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_q(
            jacobians[0]);
        J_so3_q.setZero();
        J_so3_q.block<1, 4>(0, 0) =
            -nu.transpose() / nu.norm() * J_xi_p * J_x_q * 2;
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_so3_t(
            jacobians[1]);
        J_so3_t.setZero();
        J_so3_t.block<1, 3>(0, 0) = -nu.transpose() / nu.norm() * J_xi_p;
      }
    }

    return true;
  };

public:
  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d last_point_a_,
                                     const Eigen::Vector3d last_point_b_,
                                     const double s_) {
    return new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_);
  }

  Eigen::Vector3d curr_point, last_point_a, last_point_b;
  double s;
};

class LidarPlaneFactor : public ceres::SizedCostFunction<1, 4, 3> {
  LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                   Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_,
                   double s_)
      : curr_point(curr_point_), last_point_j(last_point_j_),
        last_point_l(last_point_l_), last_point_m(last_point_m_), s(s_) {
    ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
    ljm_norm.normalize();
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);
    Eigen::Vector3d lp = q_last_curr * curr_point + t_last_curr; //   new point

    residuals[0] = (lp - last_point_j).dot(ljm_norm);

    Eigen::Vector4d q = q_last_curr.coeffs();

    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Matrix<double, 3, 4> J_x_q;
        double qx = q(0), qy = q(1), qz = q(2), qw = q(3);
        double X = curr_point(0), Y = curr_point(1), Z = curr_point(2);

        J_x_q << qy * Y + qz * Z, qw * Z + qx * Y - 2 * qy * X,
            -qw * Y + qx * Z - 2 * qz * X, qy * Z - qz * Y,
            -qw * Z - 2 * qx * Y + qy * X, qx * X + qz * Z,
            qw * X + qy * Z - 2 * qz * Y, -qx * Z + qz * X,
            qw * Y - 2 * qx * Z + qz * X, -qw * X - 2 * qy * Z + qz * Y,
            qx * X + qy * Y, qx * Y - qy * X;

        // J_x_q << 2 * X * qx + 2 * Y * qy + 2 * Z * qz,
        //     -2 * X * qy + 2 * Y * qx + 2 * Z * qw,
        //     -2 * X * qz - 2 * Y * qw + 2 * Z * qx,
        //     2 * X * qw - 2 * Y * qz + 2 * Z * qy,
        //     2 * X * qy - 2 * Y * qx - 2 * Z * qw,
        //     2 * X * qx + 2 * Y * qy + 2 * Z * qz,
        //     2 * X * qw - 2 * Y * qz + 2 * Z * qy,
        //     2 * X * qz + 2 * Y * qw - 2 * Z * qx,
        //     2 * X * qz + 2 * Y * qw - 2 * Z * qx,
        //     -2 * X * qw + 2 * Y * qz - 2 * Z * qy,
        //     2 * X * qx + 2 * Y * qy + 2 * Z * qz,
        //     -2 * X * qy + 2 * Y * qx + 2 * Z * qw;
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_q(
            jacobians[0]);
        J_so3_q.setZero();
        J_so3_q.block<1, 4>(0, 0) = ljm_norm.transpose() * J_x_q * 2;
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_so3_t(
            jacobians[1]);
        J_so3_t.setZero();
        J_so3_t.block<1, 3>(0, 0) = ljm_norm.transpose();
      }
    }
    return true;
  }

public:
  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d last_point_j_,
                                     const Eigen::Vector3d last_point_l_,
                                     const Eigen::Vector3d last_point_m_,
                                     const double s_) {
    // return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
    //     new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_,
    //                          last_point_m_, s_)));

    return new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_,
                                last_point_m_, s_);
  }

  Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
  Eigen::Vector3d ljm_norm;
  double s;
};

struct LidarPlaneNormFactor {

  LidarPlaneNormFactor(Eigen::Vector3d curr_point_,
                       Eigen::Vector3d plane_unit_norm_,
                       double negative_OA_dot_norm_)
      : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
        negative_OA_dot_norm(negative_OA_dot_norm_) {}

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
