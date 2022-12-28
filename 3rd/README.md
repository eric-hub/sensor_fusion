### 雅克比矩阵推导
1) 线特征残差
![](images/01.jpg)

2) 面特征残差
![](images/02.jpg)
### 编码实现
见代码

### EVO验证
evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xyz   
自动求导的RMSE为 18.860694   
解析求导的RMSE为 18.419905   
