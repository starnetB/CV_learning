
/*PassThrogh
 *  直通滤波
 */
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;

int main(int argc,char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //Fill in the cloud data
    cloud->width=5;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);

    for(size_t i=0;i<cloud->points.size();++i)
    {
        cloud->points[i].x=1024*rand()/(RAND_MAX+1.0f);
        cloud->points[i].x=1024*rand()/(RAND_MAX+1.0f);
        cloud->points[i].z=1024*rand()/(RAND_MAX+1.0f);
    }

    std::cerr <<"cloud before filtering:"<< std::endl;
    for(size_t i=0;i<cloud->points.size();++i)
    {
        std::cerr<<"     "<<cloud->points[i].x << " "
                 <<cloud->points[i].y<< " "
                 <<cloud->points[i].z<<std::endl;
    }

    //Creatte the filter object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);               //设置输入源
    pass.setFilterFieldName("z");          //设置过滤域名
    pass.setFilterLimits(0.0,1.0);         //设置过滤范围  (保留0-1之间的内容)
    pass.setFilterLimitsNegative(true);     //保留0-1之外的内容
    pass.filter(*cloud_filtered);     //设置输入容器

    std::cerr <<"Cloud after filtering: "<<std::endl;
    for(size_t i=0;i<cloud_filtered->points.size();++i)
    {
        std::cerr << "    " << cloud_filtered->points[i].x << " "
                  << cloud_filtered->points[i].y << " "
                  << cloud_filtered->points[i].z << std::endl;
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //这里会一直阻塞知道点云被渲染
    viewer.showCloud(cloud);
    while(!viewer.wasStopped()){}
    return 0;
}