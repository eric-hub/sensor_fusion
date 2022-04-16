## 跑通提供的工程框架
项目使用环境为双系统
*  git clone [here](https://github.com/AlexGeControl/Sensor-Fusion-for-Localization-Courseware)
*  切换分支
   git checkout 02-lidar-odometry-basic
*  将第二章代码复制到src中
*  编译 catkin_make -j8
*  修改config.yaml 的data_path 为本地项目路径，registration_method 为NDT
*  启动项目
   roslaunch lidar_localization front_end.launch
*  启动kitti数据集
   rosbag play -r 2 kitti_2011_10_03_drive_0027_synced.bag
*  效果图如下   
![01.png](images/01.png)
*  调用地图保持与地图显示 rosservice call /save_map   
![02.png](images/02.png)
---
## 使用evo
命令如下：<br>
evo_rpe kitti ground_truth.txt laser_odom.txt -r trans_part --delta 100 --plot --plot_mode xyz<br>
evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xyz<br>
指标:   
max：表示最大误差；   
mean：平均误差；   
median：误差中位数；   
min：最小误差；   
rmse：均方根误差；   
sse：和方差、误差平方和；   
std：标准差   
NDT evo_rpe图表:<br>
![ndt_raw.png](images/ndt_raw.png)
![ndt_map.png](images/ndt_map.png)<br>
NDT evo_ape图表:<br>
![ndt_ape_raw.png](images/ndt_ape_raw.png)
![ndt_ape_map.png](images/ndt_ape_map.png)<br>
ICP evo_rpe图表:<br>
![icp_raw.png](images/icp_raw.png)
![icp_map.png](images/icp_map.png)<br>
ICP evo_ape图表:<br>
![icp_ape_raw.png](images/icp_ape_raw.png)
![icp_ape_map.png](images/icp_ape_map.png)<br>

 
| 方法 | max | mean | median | min | rmse | sse | std |
| ---- | --- | ---- | ------ | --- | ---- | --- | --- |
evo_rpe: 
| NDT  | 2.237346   | 0.840434  | 0.772737  | 0.145416 | 0.949441  | 40.564705     | 0.441712  |
| ICP  | 277.272412 | 59.495329 | 28.539019 | 0.348698 | 92.620375 | 386034.025493 | 70.984785 |
evo_ape:  
| NDT  | 67.959073  | 20.691909 | 14.167748 | 0.000002 | 27.246126 | 3371760.051167 | 17.725583 |
| ICP  | 1046.451926 | 365.245545 | 334.865135 | 0.000001 | 478.340314 | 1039481358.630141  | 308.877238 |
---
### 代码实现
SICP代码实现[here](lidar_localization/src/models/registration/sicp/sicp_registration.cpp#L60)<br>
ICP_SVD实现:<br>
ScanMatch[here](lidar_localization/src/models/registration/icp_svd_registration.cpp#L85)<br>
GetCorrespondence[here](lidar_localization/src/models/registration/icp_svd_registration.cpp#L134)<br>
GetTransform[here](lidar_localization/src/models/registration/icp_svd_registration.cpp#L166)<br>

evo测试:   
SICP evo_rpe图表:<br>
![sp_raw.png](images/sp_raw.png)
![sp_map.png](images/sp_map.png)<br>
SICP evo_ape图表:<br>
![sp_a_raw.png](images/sp_a_raw.png)
![sp_a_map.png](images/sp_a_map.png)<br>

ICP_SVD evo_rpe图表:<br>
![is_raw.png](images/is_raw.png)
![is_map.png](images/is_map.png)<br>
ICP_SVD evo_ape图表:<br>
![is_a_raw.png](images/is_a_raw.png)
![is_a_map.png](images/is_a_map.png)<br>
汇总evo测试结果：   

| 方法 | max | mean | median | min | rmse | sse | std |
| ---- | --- | ---- | ------ | --- | ---- | --- | --- |
evo_rpe: 
| NDT  | 2.237346   | 0.840434  | 0.772737  | 0.145416 | 0.949441  | 40.564705     | 0.441712  |
| ICP  | 277.272412 | 59.495329 | 28.539019 | 0.348698 | 92.620375 | 386034.025493 | 70.984785 |
| ICP_SVD  | 2.295545   | 0.888383  | 0.895189  | 0.169607 | 0.963945  | 41.813549     | 0.374119  |
| SICP  | 133.268038 | 40.546010 | 25.143353 |2.101532 | 	52.001375 | 121686.433481 | 32.560160 |
evo_ape:  
| NDT  | 67.959073  | 20.691909 | 14.167748 | 0.000002 | 27.246126 | 3371760.051167 | 17.725583 |
| ICP  | 1046.451926 | 365.245545 | 334.865135 | 0.000001 | 478.340314 | 1039481358.630141  | 308.877238 |
| ICP_SVD  | 29.337443   | 16.785500  | 15.745290  |0.000001 |18.330628  | 1526502.088425     | 7.366065  |
| SICP  | 641.696804 | 275.404611 | 254.550631 | 0.000001 | 324.705382 | 478984776.546303 | 172.005480 |

总结：   
在分段统计中NDT与ICP_SVD差不多,但它俩明显优于SICP与ICP。在整体轨迹误差中,ICP_SVD稍微优于NDT,同样它俩明显优于SICP与ICP。虽然测试中SICP优于ICP,但SICP分析速度太慢。再目前数据情况下,建议NDT或ICP_SVD
