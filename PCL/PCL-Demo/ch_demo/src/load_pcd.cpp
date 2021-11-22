//我们可以对一个点云进行反序列化操作，将之保存到PointCloud对象中：load_pcd.cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc,char **argv)
{
    // 准备pcl::PointXYZ类型的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 将pcd中的数据加载到cloud中
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("./data/bunny.pcd",*cloud)==-1) //* load the file
    {
        PCL_ERROR("Couldn`t read file bunny.pcd\n");
        return(-1);
    }
    std::cout<<"Load"<<cloud->width*cloud->height
            <<"data points from test_pcd.pcd with the following fields:"
            <<std::endl;
    
    for(size_t i=0;i<cloud->points.size();i++)
    {
        std::cout<<"    " << cloud->points[i].x
                 <<" " << cloud->points[i].y
                 <<" " << cloud->points[i].z << std::endl;
    }
    return (0);

}