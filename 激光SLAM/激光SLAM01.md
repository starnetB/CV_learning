# 2D激光SLAM的帧间匹配方法
* PI(ICP)点云的点线匹配
* CSM（Correlation Scan Match)
* State of Art:CSM!梯度优化(Cartographer方法)

# 2D激光SLAM的回环检测方法
* Scan-to-Map
* Map-to-Map
* Branch and Bound & Lazy Decision(延时决策，这里我们的CartoGrapher没有使用，可以考虑以下)


# Filter-based
* EKF-SLAM
* FastSLAM
* Gmapping(里程计的性能会极大的影响Gmapping的效果，同时大型地图建图能力不太好)
* Optimal RBPF(对GMapping的一种改进)
  
# Graph-based
* Globally Consistent Range Scan For Environment Mapping----97
* Incremental Mapping of Large Cyclic Enviroments
* Karto SLAM（经过第一个的图优化的方法)
* Cartographer-------------------16年出来