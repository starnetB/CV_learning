#include <iostream>
//#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv) {

    // 创建PointCloud的智能指针
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // 加载pcd文件到cloud
    pcl::io::loadPCDFile("./data/pcl_logo.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //这里会一直阻塞直到点云被渲染
    viewer.showCloud(cloud);

    // 循环判断是否退出
    while (!viewer.wasStopped()) {
        // 你可以在这里对点云做很多处理
    }
    return 0;
}