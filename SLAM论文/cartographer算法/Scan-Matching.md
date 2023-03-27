# 1. 概念
* SLAM:它根据两帧扫描数据，基于环境的外形和扫描匹配来计算刚体变换。
* 仅仅基于里程计传感器（Odometry Sensors provided by wheel encoders）或IMU传感器创建的地图是不准确的，因为里程计传感器存在累计误差，所以LDS(Laser Distance Sensor)使用Scan-Matching/Registration算法使新扫描的激光数据与已有地图或激光扫描数据重新对齐，从而获得更加准确的机器人位姿(Pose)和地图(Map)。
* Scan-Matching问题庙速
1. SLAM定位方法
* SLAM定位方法流派
  * 概率方法
    * EKF
    * Particle filter
    * Maximum Likelihood
  * 匹配方法
    * ICP以及各种变体
    * Scan-to-Scan
    * Scan-to-Map
    * Feature-based
    * RANSAC-for-outlier-rejection
    * Correlative-matching
    * Iterative-Dual-Correspondence


## Scan-to-Scan
ICP？特征点匹配？，怎么去完成，还是不需要去完成？