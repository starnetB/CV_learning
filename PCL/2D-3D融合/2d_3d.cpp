#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

//通过双重循环遍历   
/**
     * 将彩色图和深度图合并成点云
     * @param matrix 相机内参矩阵3x3
     * @param rgb    彩色图
     * @param depth  深度图
     * @param cloud  输出点云
     */
static void convert(Mat &matrix, Mat &rgb, Mat &depth, PointCloud::Ptr &cloud)
{
    //相机内参数
    double camera_fx=matrix.at<double>(0,0);
    double camera_fy=matrix.at<double>(1,1);
    double camera_cx=matrix.at<double>(0,2);
    double camera_cy=matrix.at<double>(1,2);

    cout<<"fx:   "<<camera_fx<<endl;
    cout<<"fy:   "<<camera_fy<<endl;
    cout<<"cx:   "<<camera_cx<<endl;
    cout<<"cy:   "<<camera_cy<<endl;

    //遍历深度图
    for (int v = 0; v < depth.rows; v++)
        for (int u = 0; u < depth.cols; u++) {
            //获取深度图中(m,n)处的值
            ushort d=depth.ptr<ushort>(v)[u];
            //d 可能没有值，若如此，跳过此点
            if(isnan(d)&&abs(d)<0.0001)
                continue;
            //d 存在值，则向点云增加一个点
            PointT p;
            
            //计算这个点的空间坐标
            p.z=double(d)/1000;
            p.x=(u-camera_cx)*p.z/camera_fx;
            p.y=(v-camera_cy)*p.z/camera_fy;


            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            Vec3b bgr=rgb.at<Vec3b>(v,u);
            p.b=bgr[0];
            p.g=bgr[1];
            p.r=bgr[2];

            //把p加入到点云中
            cloud->points.push_back(p);
            //cout<<cloud->points.size()<<endl;
        }

        //设置并保持点云
        cloud->height=1;
        cloud->width=cloud->points.size();
        cout<<"point cloud size =  "<<cloud->points.size()<<endl;
        cloud->is_dense=false;       
}

int main(){
    cv::Mat cameraMatrix;  //从文件加载相机内参
    cv::Mat rgb;           //从相机得到RGB彩色图
    cv::Mat depth;         //从相机得到depth深度图
    PointCloud::Ptr pCloud=PointCloud::Ptr(new PointCloud);
    convert(cameraMatrix,rgb,depth,pCloud);
}