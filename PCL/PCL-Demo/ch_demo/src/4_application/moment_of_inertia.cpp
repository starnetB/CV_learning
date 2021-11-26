#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

int main(int argc,char** argv)
{
    if(argc!=2)
        return (0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
    {
        return (-1);
    }

    //创建惯性矩阵估算对象，设置输入点云，并进行计算
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;

    pcl::PointXYZ  min_point_AABB;
    pcl::PointXYZ  max_point_AABB;
    pcl::PointXYZ  min_point_OBB;
    pcl::PointXYZ  max_point_OBB;
    pcl::PointXYZ  position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value,middle_value,minor_value;
    Eigen::Vector3f major_vector,middle_vector,minor_vector;
    Eigen::Vector3f mass_centor;

    //
}

