//
// Created by ty on 20-5-28.
//

#include <pcl/point_types.h>
#include <pcl/features/pfh.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

int main()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::io::loadPCDFile("./data/target.pcd", *cloud);
    pcl::io::loadPCDFile("./data/bunny.pcd", *cloud);

    // estimate normals ------------------------------------------------------------- 计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for normal estimation.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    //normalEstimation.setIndices()
    normalEstimation.setInputCloud(cloud);

    // For every point, use all neighbors in a radius of 3cm.
    normalEstimation.setRadiusSearch(0.03);

    // A kd-tree is a data structure that makes searches efficient. More about it later.
    // The normal estimation object will use it to find nearest neighbors.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    // Calculate the normals.
    normalEstimation.compute(*normals);

    // Create the PFH estimation class, and pass the input dataset+normals to it ------计算PFH直方图
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);

    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);
    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    pfh.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
    // Use all neighbors in a sphere of radius 5cm
    // 使用一个半径为5厘米的球体，作为搜索邻域
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    // 重点： 半径必须要比用于估算法向量的邻域半径要大

    pfh.setRadiusSearch(0.08);

    // Compute the features
    pfh.compute(*pfhs);

    unsigned long size = pfhs->points.size();
    for (int j = 0; j < size; ++j) {
        pcl::PFHSignature125 &signature125 = pfhs->points[j];
        float* h = signature125.histogram;

        printf("%d: %f,%f,%f \n", j, h[1], h[2], h[3]);
    }

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.01, "normals");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}