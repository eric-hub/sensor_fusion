#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

struct pose {
    double timestamp;
    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
};

pose pose_gt;
pose pose_esti;

std::ofstream gt;
std::ofstream esti;

double stamp_gt = 0;
double stamp_ins = 0;

double stamp_gt_init = 0;
double stamp_esti_init = 0;

int flag_gt = 1;
int flag_esti = 1;

bool createFile(std::ofstream& ofs, std::string path) {
    ofs.open(path, std::ios::out);
    if (!ofs) {
        std::cout << "open path error" << std::endl;
        return false;
    }

    return true;
}
/*write to txt, fomat TUM*/
void writeText(std::ofstream& ofs, pose data) {
    ofs << std::fixed << data.timestamp << " " << data.pos.x() << " " << data.pos.y() << " " << data.pos.z() << " "
        << data.q.x() << " " << data.q.y() << " " << data.q.z() << " " << data.q.w() << std::endl;
}

void estiCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    if (flag_esti) {
        stamp_esti_init = msg->header.stamp.toSec();
        flag_esti = 0;
    }

    pose_esti.timestamp = msg->header.stamp.toSec() - stamp_esti_init;

    pose_esti.pos.x() = msg->pose.pose.position.x;
    pose_esti.pos.y() = msg->pose.pose.position.y;
    pose_esti.pos.z() = msg->pose.pose.position.z;

    pose_esti.q.w() = msg->pose.pose.orientation.w;
    pose_esti.q.x() = msg->pose.pose.orientation.x;
    pose_esti.q.y() = msg->pose.pose.orientation.y;
    pose_esti.q.z() = msg->pose.pose.orientation.z;

    writeText(esti, pose_esti);
}

void gtCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    if (flag_gt) {
        stamp_gt_init = msg->header.stamp.toSec();
        flag_gt = 0;
    }

    pose_gt.timestamp = msg->header.stamp.toSec() - stamp_gt_init;

    pose_gt.pos.x() = msg->pose.pose.position.x;
    pose_gt.pos.y() = msg->pose.pose.position.y;
    pose_gt.pos.z() = msg->pose.pose.position.z;

    pose_gt.q.w() = msg->pose.pose.orientation.w;
    pose_gt.q.x() = msg->pose.pose.orientation.x;
    pose_gt.q.y() = msg->pose.pose.orientation.y;
    pose_gt.q.z() = msg->pose.pose.orientation.z;
    writeText(gt, pose_gt);
}

int main(int argc, char** argv) {
    char path_gt[] = "/home/eric/fusion_work/src/imu_integration/evo/gt.txt";
    char path_esti[] = "/home/eric/fusion_work/src/imu_integration/evo/esti.txt";

    std::cout << "启动evaluate_node..." << std::endl;
    createFile(gt, path_gt);
    createFile(esti, path_esti);

    ros::init(argc, argv, "evaluate_node");

    ros::NodeHandle nh;

    ros::Subscriber sub_gt = nh.subscribe("/pose/ground_truth", 1000, gtCallBack);
    ros::Subscriber sub_esti = nh.subscribe("/pose/estimation", 1000, estiCallBack);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    gt.close();
    esti.close();

    return 0;
}
