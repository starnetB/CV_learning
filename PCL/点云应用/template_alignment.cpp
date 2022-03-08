#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>   

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PCLHandler;

class FeatureCloud {
public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    //计算特征点的描FPFH描述子
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud() :
            search_method_xyz_(new SearchMethod),
            normal_radius_(0.02f),
            feature_radius_(0.02f) {}
    
    ~FeatureCloud() {}

    // Process the given cloud
    void setInputCloud(PointCloud::Ptr xyz) {
        xyz_ = xyz;
        processInput();
    }
    
    // Load and process the cloud in the given PCD file
    void loadInputCloud(const std::string &pcd_file) {
        xyz_ = PointCloud::Ptr(new PointCloud);
        pcl::io::loadPCDFile(pcd_file, *xyz_);
        processInput();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr  getPointCloud() const {
        return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals() const {
        return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures() const {
        return (features_);
    }  //获取特征描述子
protected:
    // Compute the surface normals and local features
    void processInput() {
        computeSurfaceNormals();
        computeLocalFeatures();
    }

    // Compute the surface normals
    void  computeSurfaceNormals() {
        // 创建表面法向量
        normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

        //计算表面法向量  
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud(xyz_);
        norm_est.setSearchMethod(search_method_xyz_);
        norm_est.setRadiusSearch(normal_radius_);
        norm_est.compute(*normals_);
    }

    // Compute the local feature descriptors
    /**
     * 根据表面法向量 计算本地特征描述
     */
    void   computeLocalFeatures() {
        features_ = LocalFeatures::Ptr(new LocalFeatures);
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(xyz_);
        fpfh_est.setInputNormals(normals_);
        fpfh_est.setSearchMethod(search_method_xyz_);
        fpfh_est.setRadiusSearch(feature_radius_);
        fpfh_est.compute(*features_);
    }

private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_; // 快速点特征直方图 Fast Point Feature Histogram
    SearchMethod::Ptr search_method_xyz_; // KDTree方法查找邻域

    // Parameters
    float normal_radius_;
    float feature_radius_;
}