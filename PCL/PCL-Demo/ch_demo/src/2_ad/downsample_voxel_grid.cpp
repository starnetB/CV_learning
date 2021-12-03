// 降采样VoxelGrid 

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc,char **argv)
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    //从文件读取点云图
    //File in the cloud data
    pcl::PCDReader reader;
    //Replace the path below with the path where you saved your file
    reader.read("./data/table_scene_lms400.pcd",*cloud); 
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")."<<std::endl;

    // 创建一个长宽高分别是1cm的体素过滤器，cloud作为输入数据，cloud_filtered作为输出数据
    float leftsize=0.01f;  //pcl中标准长度以m作为单位
    // Create the filtering object

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leftsize,leftsize,leftsize);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
    //将结果输出到文件
    pcl::PCDWriter writer;
    writer.write("./data/table_scene_lms400_downsampled.pcd", *cloud_filtered);
    return (0);
}