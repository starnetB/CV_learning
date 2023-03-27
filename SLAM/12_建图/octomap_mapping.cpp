#include <iostream>
#include <fstream>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <octomap/octomap.h>    // for octomap 

#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings

int main(int argc,char **argv){
    vector<cv::Mat> colorImgs,depthImgs;
    vector<Eigen::Isometry3d> poses;

    ifstream fin("./data/pose.txt");
    if(!fin){
        cerr<<"cannot find pose file"<<endl;
        return 1;
    }

    for(int i=0;i<5;i++){
        boost::format fmt("./data/%s/%d.%s"); //图像文件格式
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "png").str(), -1)); // 使用-1读取原始图像

        double data[7]={0};
        for(int i=0;i<7;i++){
            fin>>data[i];
        }

        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    //计算点云并拼接
    //相机内参 
    double cx=319.5;
    double cy=239.5;
    double fx=481.2;
    double fy=-480.0;
    double depthScale=5000.0;

    cout << "正在将图像转换为 Octomap ..." << endl;

    //octomap tree
    //参数为分别率,这里最小的分辨率是0.1m,也就是八叉树最小的网格是0.1m，
    //tree.insertPointCloud函数会将，cloud出入tree中，在插入过程中
    //cloud 会根据这个分辨率进行体素滤波，那么这边八叉树的体素滤波的叶子节点就是0.01m
    //此外 OcTree的默认构造里面，固定层数为16层，
    //因此这个数值会影响八叉树的从中心开始的涵盖范围，也就是八叉数的外轮廓
    //0.01  更节点的最大范围就是327左右
    //如果这个分辨率是0.1 固定16层的八叉树地图就是跟节点的范围就是3270左右
    octomap::OcTree tree(0.01); 

    //我们可以改变tree的最大层次,d

    for (int i = 0; i < 5; i++) {
        cout << "转换图像中: " << i + 1 << endl;
        cv::Mat color=colorImgs[i];
        cv::Mat depth=depthImgs[i];
        Eigen::Isometry3d T=poses[i];

        octomap::Pointcloud cloud; //the point cloud in octomap

        for (int v=0;v<color.rows;v++)
            for(int u=0;u<color.cols;u++){
                unsigned int d=depth.ptr<unsigned short>(v)[u]; //深度图
                if(d==0) continue;
                Eigen::Vector3d point;
                point[2]=double(d)/depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;
                // 将世界坐标系的点放入点云
                cloud.push_back(pointWorld[0], pointWorld[1], pointWorld[2]);
            }

        // 将点云存入八叉树地图，给定原点，这样可以计算投射线
        // 我认为分别给定点云以及点云中心，方便在更新时，根据各个点云的原点，重新更新八叉树的投射中心
        tree.insertPointCloud(cloud, octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
    }
    //以下部分代码是将pcl中的点云直接加到tree中的方法
    /*
    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中
         tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }*/
    
    // 更新中间节点的占据信息并写入磁盘
    tree.updateInnerOccupancy();
    cout<<"saving octomap ...  "<<endl;
    tree.writeBinary("octomap.bt");
    return 0;
}