#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>

int main(int argc,char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/bunny.pcd",*cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_milk(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("./data/milk_color.pcd",*cloud_milk);

    //创建PCLVisualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    

    //设置背景色为灰色(非必须)
    viewer->setBackgroundColor(0.05,0.05,0.05,0);

    //添加一个彩色点云配置,也可以去掉single_color参数不设置)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud,0,255,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,single_color,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sample cloud");  //点的大小

    //// 再添加一个彩色点云及配置
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_milk);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_milk,rgb,"sample cloud milk");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample cloud milk");

    //添加一个0.5倍缩放的坐标系（非必须）
    viewer->addCoordinateSystem(0.5);

    // 直到窗口关闭才结束循环
    while(!viewer->wasStopped()){
        // 每次循环调用内部的重绘函数
        viewer->spinOnce();

    }
    return 0;

}