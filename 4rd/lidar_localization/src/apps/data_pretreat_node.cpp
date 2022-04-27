/*
 * @Description: 数据预处理的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include "glog/logging.h"
#include <ros/ros.h>

#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"
#include "lidar_localization/global_defination/global_defination.h"
#include <lidar_localization/optimizeMap.h>

using namespace lidar_localization;

bool _need_save_origin = false;

bool save_origin_callback(optimizeMap::Request &request,
                          optimizeMap::Response &response) {
  _need_save_origin = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "data_pretreat_node");
  ros::NodeHandle nh;

  std::string cloud_topic;
  nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

  // subscribe to
  // a. raw Velodyne measurement
  // b. raw GNSS/IMU measurement
  // publish
  // a. undistorted Velodyne measurement
  // b. lidar pose in map frame
  std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr =
      std::make_shared<DataPretreatFlow>(nh, cloud_topic);

  ros::ServiceServer service =
      nh.advertiseService("save_origin", save_origin_callback);

  // pre-process lidar point cloud at 100Hz:
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    data_pretreat_flow_ptr->Run();
    if (_need_save_origin) {
      data_pretreat_flow_ptr->saveOrigin();
      _need_save_origin = false;
    }
    rate.sleep();
  }

  return 0;
}