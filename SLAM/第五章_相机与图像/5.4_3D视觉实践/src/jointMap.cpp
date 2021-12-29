#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>  //for formaing strings
#include <sophus/se3.hpp>   //李群与李代库
#include <pangolin/pangolin.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;

//一个se李代的容器
typedef vector<Sophus::SE3d,Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double,6,1> Vector6d;
//在pangolin 中画图，已写好，无需调整

//体素滤波
vector<Vector6d,Eigen::aligned_allocator<Vector6d>> down_sample_vex(
                vector<Vector6d,Eigen::aligned_allocator<Vector6d>> pointcloud);
void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud)
{
    if(pointcloud.empty())
    {
        cerr<<"Point cloud is empty!"<<endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer",1024,768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC1_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
        pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1.0,0.0)
    );

    pangolin::View &d_cam=pangolin::CreateDisplay()
        .SetBounds(0.0,1.0,pangolin::Attach::Pix(175),1.0,-1024.0f/768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

        while (pangolin::ShouldQuit()==false){
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);

            glPointSize(2);
            glBegin(GL_POINTS);
            for (auto &p:pointcloud){
                glColor3d(p[3]/255.0,p[4]/255.0,p[5]/255.0);
                glVertex3d(p[0],p[1],p[2]);
            }
            glEnd();
            pangolin::FinishFrame();
            usleep(5000);  //sleep 5 ms
        }
        return ;
}

int main(int argc,char **argv){
    vector<cv::Mat> colorImgs,depthImgs;  //彩色图和深度图
    TrajectoryType poses;                   //se(3)李代计算容器,但没计算之前还是李群空间
    ifstream fin("./pose.txt");
    if(!fin){
        cerr<<"请在有pose.txt的目录下运行此程序" << endl;
        return 1;
    }

    for (int i=0;i<5;i++){
        boost::format fmt222("./%s/%d.%s");  //图像文件格式
        colorImgs.push_back(cv::imread((fmt222% "color"%(i+1)%"png").str()));
        depthImgs.push_back(cv::imread((fmt222% "depth"%(i+1)%"pgm").str(),-1));   //使用-1 读取原始图像，也就是不对图像进行任何数据转换

        double data[7]={0};
        for (auto &d:data)
            fin>>d;
        Sophus::SE3d pose(Eigen::Quaterniond(data[6],data[3],data[4],data[5]),    //W,X,Y,Z 
                          Eigen::Vector3d(data[0],data[1],data[2])); 
        poses.push_back(pose);
    }
        //计算点云并拼接
        //相机内参

        double cx=325.5;
        double cy=253.5;
        double fx=518.0;
        double fy=519.0;
        double depthScale=1000.0;
        vector<Vector6d,Eigen::aligned_allocator<Vector6d>> pointcloud;
        //reserve的作用是更改vector的容量（capacity），使vector至少可以容纳n个元素。
        //如果n大于vector当前的容量，reserve会对vector进行扩容。其他情况下都不会重新分配vector的存储空间
        pointcloud.reserve(1000000);  
        
        for(int i=0;i<5;i++){
            cout <<"转换图像中： "<<i+1<<endl;
            cv::Mat color=colorImgs[i];
            cv::Mat depth=depthImgs[i];
            Sophus::SE3d T=poses[i];
            for(int v=0;v<color.rows;v++)
                for(int u=0;u<color.cols;u++){
                    
                    unsigned int d=depth.ptr<unsigned short>(v)[u]; //深度值
                    if(d==0) continue;  //为0表示没有测量到
                    Eigen::Vector3d point; 
                    point[2]=double(d)/depthScale;   //毫米变成米？
                    point[0]=(u-cx)*point[2]/fx;
                    point[1]=(v-cy)*point[2]/fy;     //相机坐标系下的xy轴
                    Eigen::Vector3d  pointWorld=T*point;  //将坐标系转到世界坐标系下
                    
                    Vector6d p;             //一个6d的点 ，前三个时位置信息，后面三个时颜色信息
                    p.head<3>()=pointWorld;
                    p[5]=color.data[v*color.step+u*color.channels()];  //blue
                    p[4]=color.data[v*color.step+u*color.channels()+1];  //green
                    p[3]=color.data[v*color.step+u*color.channels()+2];  //red   将颜色点加到6为向量的后面三个
                    pointcloud.push_back(p);       //这边的话 xyzrgb都有了   
                }
        }
        cout<<"点云共有"<<pointcloud.size()<<"个点."<<endl;
        vector<Vector6d,Eigen::aligned_allocator<Vector6d>> pointcloud2=down_sample_vex(pointcloud);
        showPointCloud(pointcloud2);
        return 0;

}

//体素滤波
vector<Vector6d,Eigen::aligned_allocator<Vector6d>> down_sample_vex(
                vector<Vector6d,Eigen::aligned_allocator<Vector6d>> pointcloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    vector<Vector6d,Eigen::aligned_allocator<Vector6d>> outputcloud;

    cloud->width=pointcloud.size();
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);

    for(size_t i=0;i<cloud->points.size();++i)
    {
        cloud->points[i].x=pointcloud[i][0];
        cloud->points[i].y=pointcloud[i][1];
        cloud->points[i].z=pointcloud[i][2];
        cloud->points[i].r=pointcloud[i][3];
        cloud->points[i].g=pointcloud[i][4];
        cloud->points[i].b=pointcloud[i][5];
    }

    // 创建一个长宽高分别是1cm的体素过滤器，cloud作为输入数据，cloud_filtered作为输出数据
    float leftsize=0.01f;  //pcl中标准长度以m作为单位
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);

    sor.setLeafSize(leftsize,leftsize,leftsize);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
    
    for (int i=0;i<cloud_filtered->points.size();i++)
    {
        Vector6d newpose;
        newpose[0]=cloud_filtered->points[i].x;
        newpose[1]=cloud_filtered->points[i].y;
        newpose[2]=cloud_filtered->points[i].z;
        newpose[3]=cloud_filtered->points[i].r;
        newpose[4]=cloud_filtered->points[i].g;
        newpose[5]=cloud_filtered->points[i].b;
        outputcloud.push_back(newpose);
    }
    return outputcloud;
}   


    

    



   

    
    
    
    