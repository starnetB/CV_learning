#include<pcl/visualization/cloud_viewer.h>
#include<iostream>
#include<pcl/io/pcd_io.h>

int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{
    //// 设置背景色为粉红色
    viewer.setBackgroundColor(1.0,0.5,1.0);
    pcl::PointXYZ o;
    o.x=1.0;
    o.y=0;
    o.z=0;
    //    添加一个圆心为o，半径为0.25m的球体
    viewer.addSphere(o,0.25,"sphere",0);
    std::cout<<"i only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
    static unsigned count=0;
    std::stringstream ss;
    ss<<"once per viewer loop:"<<count++;
    // 每次刷新时，移除text，添加新的text
    viewer.removeShape("text",0);
    viewer.addText(ss.str(),200,300,"text",0);

    //FIXME: possible race condition here:
    user_data++;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile("./data/pcl_logo.pcd",*cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //这里会一直阻塞直到点云被渲染
    viewer.showCloud(cloud);

    // 只会调用一次 (非必须)
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    // 每次可视化迭代都会调用一次（频繁调用） (非必须)
    viewer.runOnVisualizationThread (viewerPsycho);

    while(!viewer.wasStopped())
    {
        user_data++;
    }
    return 0;
}