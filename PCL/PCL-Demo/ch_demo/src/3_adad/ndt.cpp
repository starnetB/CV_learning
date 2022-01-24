#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>  // 滤波文件头

#include <pcl/visualization/pcl_visualizer.h>   //可视化头文件

using namespace std::chrono_literals;

int main(int argc,char **argv){
    // Loading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("./data/room_scan1.pcd", *target_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    // Loading second scan of room from new perspective.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("./data/room_scan2.pcd", *input_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;
    
    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);  //点云过滤之后的对象 

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;  //过滤器
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);  //体素大小
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
              << " data points from room_scan2.pcd" << std::endl;  //过滤后的大小,只有10%了

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;  //初始化NDT 求解器  

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    // 设定在迭代过程中,delta x 的收敛条件  
    ndt.setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    // 在迭代过程中,我们的步长不可以超过你设定的值,防止跳跃行为的发生  
    ndt.setStepSize(0.1);
    ////Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(1.0);  //单位是米
    //设置NDT网格结构的分辨率（VoxelGridCovariance），即设置网格化时立方体的边长，网格大小设置在NDT中非常重要，太大会导致精度不高，太小导致内存过高，并且只有两幅点云相差不大的情况才能匹配。需要调，网格的体素分辨率。它需要足够大，每个体素包含与点有关的统计数据，平均值，协方差等，统计数据作为一组多元高斯分布用来模拟点云，并且允许我们计算和优化体素内任意位置点的存在概率。该参数是与尺度最相关的，每个体素至少包含6个点，但小到足以唯一地描述环境。调的经验：resolution在追求精度的情况下最好与点密度相等或者低一个数量级,这里是激光点云，所以设了1米
    ndt.setMaximumIterations(35);

    // Setting point cloud to be aligned.
    // 这个应该是你要对齐的点云,H应该是指向他的
    ndt.setInputCloud(filtered_cloud);  //把过滤后的点云作为你要配准的点云


    // Setting point cloud to be aligned to.
    // 这个应该就是参考系的点云,也就是你要对齐的目标
    // H由target_cloud ->filtered_cloud
    // 点云转换方向,是由filtered_cloud 所在的原始坐标系指向target_cloud
    ndt.setInputTarget(target_cloud);

    // Set initial alignment estimate found using robot odometry.
    // 初始化转换H矩阵 
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
    

    //设定输出点云  
     pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

     //使用了初始变换矩阵，不使用初始变换ndt.align (*output_cloud)可以试试，仿真结果不同。进行ndt配准,计算变换矩阵。output_cloud: 存储input_cloud经过配准后的点云(由于input_cloud被极大的降采样了,因此这个数据没什么用) 。此处output_cloud不能作为最终的源点云变换，因为上面对源点云进行了滤波处理

    std::cout << "Normal Distributions Transform has converged:"
              << ndt.hasConverged() //英文翻译已收敛，此应该是收敛函数。
              << " score（分数越大，配准效果越差）： "
              << ndt.getFitnessScore()<<std::endl;//欧几里得适合度得分FitnessScore，该分数计算为从输出云到目标云中最近点的距离的平方。分数越大，准确率越低，配准效果也就越差


    //使用创建的变换对未过滤的输入点云（intput_cloud）进行变换

    //进行点云的变换主要用到的的函数是pcl::transformPointCloud，其原型pcl::transformPointCloud(const pcl::PointCloud< PointT > &  cloud_in,
    //pcl::PointCloud<PointT>& cloud_out,
    //const Eigen::Matrix4f  & transform )
    //cloud_in为源点云，cloud_out为变换后点云,transform为变换矩阵
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

    // Saving transformed input cloud.
    //保存转换后的输出点云

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color(target_cloud, 255, 0, 0);

    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped()) {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return (0);
}


//最后的效果可以砍下markdown 文档 