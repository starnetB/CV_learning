#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int getN_1(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd",*cloud)==-1) //* load the file
    {
        PCL_ERROR("Couldn`t read file bunny.pcd\n");
        return(-1);
    }

    // 创建法向量估算类，传递输入数据集
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 创建一个空的kdtree，将值传递给法向量估算对象
    // 这个tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其他的search surface）
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    //定义输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    //使用-个半径为3cm的球体中的所有邻居点
    ne.setRadiusSearch(0.03);

    //计算特征
    ne.compute(*cloud_normals);
    //cloud_normals->points.size () 输出特征的点个数应当定于输入的点个数 cloud->points.size ()

    //创建PCLVisualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud,0,255,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,single_color,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sample cloud");

    //创建PCLVisualizer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> rgb_color(cloud_normals,0,255,0);
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals,10,0.01,"features cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"features cloud");

    //coordinate
    viewer->addCoordinateSystem(0.5);

    while(!viewer->wasStopped()){
        //每次循环调用内部的重绘函数
        viewer->spinOnce();
    }
    return 0;


    //以下内容用于可视化法向量
    //int v1(0),v2(0);
    //viewer->createViewPort(0.5,0.0,1.0,0.5,v2);

}

int getN_2(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd",*cloud)==-1) //* load the file
    {
        PCL_ERROR("Couldn`t read file bunny.pcd\n");
        return -1;
    }
    // 准备一个indices索引集合，为了简单起见，我们直接使用点云的前10%的点
    std::vector<int> indices(std::floor(cloud->points.size()/10));
    for (std::size_t i=0;i<indices.size();++i) indices[i]=i;

    //创建法向量估算类，设置输入点云
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);

    //设置indices索引
    std::shared_ptr<std::vector<int>> indicesptr(new std::vector<int>(indices));
    ne.setIndices(indicesptr);

    // 创建一个空的kdtree，将值传递给法向量估算对象
    // 这个tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其他的search surface）
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    //自己对于KDtree的部分理解，如果只有自己，那么把自己变成KDtree，然后自己查询自己
    //如果设置了SearchSurface,那么把别人变成KDtree，然后自己在别人那里找邻近点

    //定义输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    //使用一个半径为3cm的球体中的所有邻近点
    ne.setRadiusSearch(0.03);

    //计算法向量特征
    ne.compute(*cloud_normals);
    // cloud_normals->points.size () 输出特征的点个数应当定于输入的点个数 cloud->points.size ()

    //创建PCLVisualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud,0,255,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,single_color,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sample cloud");

    //创建PCLVisualizer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> rgb_color(cloud_normals,0,255,0);
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals,50,.01,"features cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"features cloud");

    //coordinate
    viewer->addCoordinateSystem(0.5);

    while(!viewer->wasStopped()){
        //每次循环调用内部的重绘函数
        viewer->spinOnce();
    }
    return 0;

}
/*
void getN_3()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

  //... read, pass in or create a point cloud ...

  //... create a downsampled version of it ...
/
  // 创建法向量估算类，将降采样后的数据作为输入点云
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_downsampled);

  // 传入降采样之前的原始数据作为search surface
  ne.setSearchSurface (cloud);

  // 创建一个空的kdtree，将值传递给法向量估算对象
  // 这个tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其他的search surface）
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // 定义输出数据集
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // 使用一个半径为3cm的球体中的所有邻居点
  ne.setRadiusSearch (0.03);

  // 计算特征
  ne.compute (*cloud_normals);

  // cloud_normals->points.size()输出特征的点个数应当定于输入的点个数cloud->points.size()
}*/

int main(){
    //getN_1();
    getN_2();
}