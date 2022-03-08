#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


typedef pcl::PointXYZ PointT;

int main(int argc,char **argv){
    // All the objects needed
    pcl::PCDReader reader;                          // PCD文件读取对象
    pcl::PassThrough<PointT> pass;                  // 直通滤波器
    pcl::NormalEstimation<PointT, pcl::Normal> ne;  // 法线估算对象
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;   // 分割器
    pcl::PCDWriter writer;                                      // PCD文件写出对象
    pcl::ExtractIndices<PointT> extract;                        // 点提取对象
    pcl::ExtractIndices<pcl::Normal> extract_normals;           // 法线提取对象
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());


     // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    //Read in the cloud data 读取点云数据
    reader.read("./data/table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

    //build a passthrough filter to remove spurious Nans
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0,1.5);
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;


    //Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    //Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);        //在判断过程中，法线值占的权重
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;


    //Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);   //这里的代表，把planar，从input cloud 里面拿出来，然后得到planar

    
    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points."
              << std::endl;
    writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

    //Remove the planar inliers,extract the rest
    extract.setNegative(true)   //这边是移除平面的需要





}


