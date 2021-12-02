#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc,char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //从文件读取点云
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ>("./data/table_scene_lms400.pcd",*cloud);

    std::cerr << "Cloud before filtering: "<<std::endl;
    std::cerr << *cloud <<std::endl;

    // 创建过滤器，每个点分析计算时考虑的最近邻居个数为50个；
    // 设置标准差阈值为1，这意味着所有距离查询点的平均距离的标准偏差均大于1个标准偏差的所有点都将被标记为离群值并删除。
    // 计算输出并将其存储在cloud_filtered中

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    // 设置平均距离估计的最近邻居的数量K
    sor.setMeanK(50);
    //设置标准差阈值稀疏
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr<<"cloud after filtering: "<<std::endl;
    std::cerr<<*cloud_filtered<<std::endl;
    //将留下来的点保存到后缀为_inliers.pcd的文件中
    pcl::PCDWriter writer;
    writer.write("./data/table_scene_lms400_inliers.pcd", *cloud_filtered, false);

     // 使用个相同的过滤器，但是对输出结果取反，则得到那些被过滤掉的点，保存到_outliers.pcd文件
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ> ("./data/table_scene_lms400_outliers.pcd", *cloud_filtered, false);

    return (0);
}