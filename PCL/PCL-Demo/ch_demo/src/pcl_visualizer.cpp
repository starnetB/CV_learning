#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>

int main(int argc,char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/bunny.pcd",*cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_milk(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("./data/milk")
}