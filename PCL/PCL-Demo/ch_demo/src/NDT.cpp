#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>     //ndt配准文件头  
#include <pcl/filters/approximate_voxel_grid.h>  //滤波文件头   

#include <pcl/visualization/pcl_visualizer.h>  

using namespace std::chrono_literals;

int main(int argc,char **argv){
    //Loading first scan of room 
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
    // 过滤掉10%的内容，为了增加回归的速度  
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);  //云输出对象  
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;   
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);   
    approximate_voxel_filter.setInputCloud(input_cloud);  
    approximate_voxel_filter.filter(*filtered_cloud);  
    std::cout << "Filtered cloud contains " << filtered_cloud->size()<< " data points from room_scan2.pcd" << std::endl;

    //Initializing Normal Distributions Transform (NDT). 
    //初始化一个正太分布 配准  
    pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> ndt;

    //Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(1.0);

    // Setting max number of registration iterations
    ndt.setMaximumIterations(35);
    
    // Setting point cloud to be aligned.
    ndt.setInputSource(filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(target_cloud);

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // Calculating required rigid transform to align the input cloud to the target cloud.

}