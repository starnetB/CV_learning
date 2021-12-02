#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h> 

typedef pcl::PointXYZ PointType;

void showPointClouds(const pcl::PointCloud<PointType>::Ptr &cloud,const pcl::PointCloud<PointType>::Ptr &cloud2){
    // 创建PCLVisualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // 设置背景色为灰色
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

    // 添加一个普通点云 (可以设置指定颜色，也可以去掉single_color参数不设置)
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<PointType>(cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    // 添加一个第二个点云 (可以设置指定颜色，也可以去掉single_color2参数不设置)
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color2(cloud, 255, 0, 0);
    viewer->addPointCloud<PointType>(cloud2, single_color2, "sample cloud 2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud 2");
    viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

int main(int argc,char **argv){
    if (argc!=2)
    {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //Fill in the cloud data
    cloud->width=100;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    if(strcmp(argv[1],"-r")==0){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        //build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.4);
        outrem.setMinNeighborsInRadius(10);
        //apply filter
        outrem.filter(*cloud_filtered);
    }else if(strcmp(argv[1],"-c")==0)
    {
        //build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
        /* 
        pcl::ComparisonOps::LE //小于等于
        pcl::ComparisonOps::GT //大于
        pcl::ComparisonOps::LT //小于
        pcl::ComparisonOps::EQ //相等
        */
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z",pcl::ComparisonOps::GT,0.0)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z",pcl::ComparisonOps::LT,0.8)));
        
        //build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(cloud);
        condrem.setKeepOrganized(false);
        //condrem.setKeepOrganized(true);  是否保留原有的点云组织，保留的话去掉的点会以nan的形式出现
        //apply filter
        condrem.filter(*cloud_filtered);
    }else{
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << std::endl;
    // display pointcloud after filtering
    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " "
                  << cloud_filtered->points[i].y << " "
                  << cloud_filtered->points[i].z << std::endl;
    
    showPointClouds(cloud,cloud_filtered);

    return (0);
}