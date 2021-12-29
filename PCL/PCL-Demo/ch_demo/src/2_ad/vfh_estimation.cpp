#include <pcl/point_types.h>
#include <pcl/features/vfh.h>

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
    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    vfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute (*vfhs);

  // vfhs->size () should be of size 1*
  unsigned long size = vfhs->points.size();
    for (int j = 0; j < size; ++j) {
        pcl::VFHSignature308 &Signature308 = vfhs->points[j];
        float* h = Signature308.histogram;

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