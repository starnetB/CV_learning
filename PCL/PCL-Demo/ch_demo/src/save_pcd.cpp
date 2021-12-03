#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc,char **argv){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //Fill in the cloud data
    cloud.width=5;
    cloud.height=1;
    cloud.is_dense=false;
    cloud.points.resize(cloud.width* cloud.height);

    std::cout<<rand()<<std::endl;
    std::cout<<rand()/(RAND_MAX+1.0f)<<std::endl;
    std::cout<<1024 *rand()/(RAND_MAX+1.0f)<<std::endl;

    // 随机生成5个点
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    //pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    //下面会以二进制格式保存图片
    pcl::io::savePCDFile("test_pcd_binary.pcd",cloud,true);
    
    std::cerr<<"Saved"<<cloud.points.size()<<" data point to test_pcd.pcd"<<std::endl;

    for (size_t i=0;i<cloud.points.size();++i)
    {
        std::cerr<<"     "<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z<<std::endl;
    }
    return (0);
}   