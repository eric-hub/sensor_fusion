/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <pcl/common/transforms.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "glog/logging.h"

#include "lidar_localization/models/registration/icp_svd_registration.hpp"

namespace lidar_localization {

ICPSVDRegistration::ICPSVDRegistration(const YAML::Node &node)
    : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
  // parse params:
  float max_corr_dist = node["max_corr_dist"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();

  SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

ICPSVDRegistration::ICPSVDRegistration(float max_corr_dist, float trans_eps,
                                       float euc_fitness_eps, int max_iter)
    : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
  SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool ICPSVDRegistration::SetRegistrationParam(float max_corr_dist,
                                              float trans_eps,
                                              float euc_fitness_eps,
                                              int max_iter) {
  // set params:
  max_corr_dist_ = max_corr_dist;
  trans_eps_ = trans_eps;
  euc_fitness_eps_ = euc_fitness_eps;
  max_iter_ = max_iter;

  LOG(INFO) << "ICP SVD params:" << std::endl
            << "max_corr_dist: " << max_corr_dist_ << ", "
            << "trans_eps: " << trans_eps_ << ", "
            << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
            << "max_iter: " << max_iter_ << std::endl
            << std::endl;

  return true;
}

bool ICPSVDRegistration::SetInputTarget(
    const CloudData::CLOUD_PTR &input_target) {
  input_target_ = input_target;
  input_target_kdtree_->setInputCloud(input_target_);

  return true;
}

bool ICPSVDRegistration::ScanMatch(const CloudData::CLOUD_PTR &input_source,
                                   const Eigen::Matrix4f &predict_pose,
                                   CloudData::CLOUD_PTR &result_cloud_ptr,
                                   Eigen::Matrix4f &result_pose) {
  input_source_ = input_source;

  // pre-process input source:
  CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
  pcl::transformPointCloud(*input_source_, *transformed_input_source,
                           predict_pose);

  // init estimation:
  transformation_.setIdentity();

  //
  // TODO: first option -- implement all computing logic on your own
  //可以做均匀采样
  // do estimation:
  int curr_iter = 0;
  while (curr_iter < max_iter_) {
    // TODO: apply current estimation:
    CloudData::CLOUD_PTR current_input(new CloudData::CLOUD());
    pcl::transformPointCloud(*transformed_input_source, *current_input,
                             transformation_);
    // TODO: get correspondence:
    std::vector<Eigen::Vector3f> xs;
    std::vector<Eigen::Vector3f> ys;

    int num_corr = GetCorrespondence(current_input, xs, ys);

    // TODO: do not have enough correspondence -- break:
    if (num_corr < 10)
      break;

    // TODO: update current transform:
    Eigen::Matrix4f transformation_delta;
    GetTransform(xs, ys, transformation_delta);

    // TODO: whether the transformation update is significant:
    if (!IsSignificant(transformation_delta, trans_eps_))
      break;

    // TODO: update transformation:
    transformation_ = transformation_delta * transformation_;

    ++curr_iter;
  }

  // set output:
  result_pose = transformation_ * predict_pose;

  Eigen::Matrix3f R = result_pose.block(0, 0, 3, 3);
  Eigen::Vector3f t = result_pose.block(0, 3, 3, 1);
  Eigen::Quaternionf q(R);
  q.normalize();
  result_pose.setIdentity();
  result_pose.block(0, 0, 3, 3) = q.toRotationMatrix();
  result_pose.block(0, 3, 3, 1) = t;

  pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);

  return true;
}

size_t
ICPSVDRegistration::GetCorrespondence(const CloudData::CLOUD_PTR &input_source,
                                      std::vector<Eigen::Vector3f> &xs,
                                      std::vector<Eigen::Vector3f> &ys) {
  const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

  size_t num_corr = 0;

  // TODO: set up point correspondence
  for (size_t i = 0; i < input_source->points.size(); i++) {
    std::vector<int> pointIdx; //保存每个近邻点的索引
    std::vector<float>
        squaredDistance; //保存每个近邻点与查找 点之间的欧式距离平⽅
    if (input_target_kdtree_->nearestKSearch(input_source->points[i], 1,
                                             pointIdx, squaredDistance) > 0) {
      if (squaredDistance[0] > MAX_CORR_DIST_SQR)
        continue;
      Eigen::Vector3f x(input_target_->points[pointIdx[0]].x,
                        input_target_->points[pointIdx[0]].y,
                        input_target_->points[pointIdx[0]].z);

      Eigen::Vector3f y(input_source->points[i].x, input_source->points[i].y,
                        input_source->points[i].z);

      xs.push_back(x);
      ys.push_back(y);
      num_corr++;
    }
  }

  return num_corr;
}

void ICPSVDRegistration::GetTransform(const std::vector<Eigen::Vector3f> &xs,
                                      const std::vector<Eigen::Vector3f> &ys,
                                      Eigen::Matrix4f &transformation_) {
  const size_t N = xs.size();

  // TODO: find centroids of mu_x and mu_y:
  Eigen::Vector3f mu_x(0, 0, 0), mu_y(0, 0, 0);
  for (size_t i = 0; i < N; i++) {
    mu_x += xs[i];
    mu_y += ys[i];
  }
  //   std::cout << "-----------" << std::endl;
  //   std::cout << mu_x << std::endl;
  mu_x = mu_x / N;
  mu_y = mu_y / N;
  //   std::cout << mu_x << std::endl;

  std::vector<Eigen::Vector3f> xss;
  std::vector<Eigen::Vector3f> yss;
  for (size_t i = 0; i < N; i++) {
    xss.push_back(xs[i] - mu_x);
    yss.push_back(ys[i] - mu_y);
  }

  // TODO: build H:
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  for (size_t i = 0; i < N; i++) {
    H += yss[i] * xss[i].transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3f U = svd.matrixU();
  Eigen::Matrix3f V = svd.matrixV();

  // TODO: solve R:
  Eigen::Matrix3f R = V * U.transpose();

  // TODO: solve t:
  Eigen::Vector3f t;
  t = mu_x - R * mu_y;
  // TODO: set output:
  transformation_.setZero();
  transformation_.block(0, 0, 3, 3) = R;
  transformation_.block(0, 3, 3, 1) = t;
  transformation_(3, 3) = 1;
}

bool ICPSVDRegistration::IsSignificant(const Eigen::Matrix4f &transformation,
                                       const float trans_eps) {
  // a. translation magnitude -- norm:
  float translation_magnitude = transformation.block<3, 1>(0, 3).norm();
  // b. rotation magnitude -- angle:
  float rotation_magnitude =
      fabs(acos((transformation.block<3, 3>(0, 0).trace() - 1.0f) / 2.0f));

  return ((translation_magnitude > trans_eps) ||
          (rotation_magnitude > trans_eps));
}

} // namespace lidar_localization