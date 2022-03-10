#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/impl/mls.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointXYZRGBNormal SurfelT;
typedef pcl::PointCloud<SurfelT> SurfelCloud;
typedef pcl::PointCloud<SurfelT>::Ptr SurfelCloudPtr;

//实现了曲面重构
SurfelCloudPtr reconstructSurface(const PointCloudPtr &input,
                                  float radius,int polynomial_order)
{
    pcl::MovingLeastSquares<PointT,SurfelT> mls;  //移动最小而乘法来平滑表面
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);  //使用KD数来进行作为查询树
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(true); // 需要法线
    mls.setSqrGaussParam(radius*radius);
    mls.setPolynomialFit(polynomial_order>1);  //如果大于二阶，那么就需要使用多项式进行拟合，来提高精度
    mls.setPolynomialOrder(polynomial_order);  //设定阶
    mls.setInputCloud(input);
    SurfelCloudPtr output(new SurfelCloud);
    mls.process(*output);
    return (output);
}

//构建三角网格
pcl::PolygonMeshPtr triangularMesh(const SurfelCloudPtr &surfels)
{
    //Create search tree
    pcl::search::KdTree<SurfelT>::Ptr tree(new pcl::search::KdTree<SurfelT>);
    tree->setInputCloud(surfels);

    //Initial objectes
    //以下内容请见贪婪投影算法  


}