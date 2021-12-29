#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>

#include "sophus/se3.hpp"

using namespace Sophus;
using namespace std;

string groundtruth_file="./example/groundtruth.txt";
string estimated_file="./example/estimated.txt";

typedef vector<Sophus::SE3d,Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

void DrawTrajectory(const TrajectoryType &gt,const TrajectoryType &esti);

TrajectoryType ReadTrajectory(const string &path);

int main(int argc,char **argv){
    TrajectoryType groundtruth=ReadTrajectory(groundtruth_file);
    TrajectoryType estimated=ReadTrajectory(estimated_file);

    //确保列表不为空，确保列表长度相等
    assert(!groundtruth.empty()&&!estimated.empty());  //如果满足条件就不发出警告，如果条件不满足就发出警告
    assert(groundtruth.size()==estimated_file.size());

    //compute rmse 计算轨迹绝对误差
    double rmse=0;
    for(size_t i=0;i<estimated.size();i++){
        Sophus::SE3d p1=estimated[i],p2=groundtruth[i];
        double error=(p2.inverse()*p1).log().norm();  //求误差之后，求L2范式
        rmse +=error*error;
    }
    rmse=rmse/double(estimated.size());
    rmse=sqrt(rmse);

    cout<<"RMSE = "<<rmse<<endl;

    DrawTrajectory(groundtruth,estimated);  //画出所有轨迹
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
// create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
/**
* glEnable(GL_DEPTH_TEST)：
*  用来开启更新深度缓冲区的功能，也就是，如果通过比较后深度值发生变化了，会进行更新深度缓冲区的操作。
* 启动它，OpenGL就可以跟踪再Z轴上的像素，这样，它只会再那个像素前方没有东西时，才会绘画这个像素。
* 在做绘画3D时，这个功能最好启动，视觉效果比较真实。
* */
  glEnable(GL_DEPTH_TEST);
/**
* Blend 混合是将源色和目标色以某种方式混合生成特效的技术。混合常用来绘制透明或半透明的物体。
* 在混合中起关键作用的α值实际上是将源色和目标色按给定比率进行混合，以达到不同程度的透明。
* α值为0则完全透明，α值为1则完全不透明。混合操作只能在RGBA模式下进行，颜色索引模式下无法指定α值。
* 物体的绘制顺序会影响到OpenGL的混合处理。*/
  glEnable(GL_BLEND);
/**
* 如果设置了glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
* 则表示源颜色乘以自身的alpha 值，目标颜色乘以1.0减去源颜色的alpha值，这样一来，
* 源颜色的alpha值越大，则产生的新颜色中源颜色所占比例就越大，而目标颜色所占比例则减小。
* 这种情况下，我们可以简单的将源颜色的alpha值理解为“不透明度”。这也是混合时最常用的方式。*/
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//定义投影和初始模型视图矩阵
/**
* ProjectionMatrix()
* 设置相机内参，以及能看到的最大最小距离
* 如ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000)
* 表示相机分辨率，焦距，相机光心，最小最大距离*/

/**ModelViewLookAt()
* 设置观看视角，pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
* 的意思是在世界坐标(0，-0.1，-1.8)处观看坐标原点（0,0,0）并设置Y轴向上*/
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

//SetBounds 边界设置，最后一个是长宽比 -1024.0f/768.0f
//前四个参数表示视图在视窗中的位置(这么写是在屏幕中央)
  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while (pangolin::ShouldQuit() == false) {
    //先清空缓冲区
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //激活相机
    d_cam.Activate(s_cam);  

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);  //清理颜色，也就是背景颜色

    glLineWidth(2);  //设置线宽度
    //将groundtrue 的每个点连接起来
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
      glBegin(GL_LINES);
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    //将estimated的每个点连接起来
    for (size_t i = 0; i < esti.size() - 1; i++) {
      glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
      glBegin(GL_LINES);
      auto p1 = esti[i], p2 = esti[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();  //绘图完成
    usleep(5000);   // sleep 5 ms
  }
}

TrajectoryType ReadTrajectory(const string &path){
    ifstream fin(path);
    TrajectoryType trajectory;
    if (!fin){
        cerr<<"trajectory  "<<path<<" not found" <<endl;
        return trajectory;
    }

    while(!fin.eof()){
        double time,tx,ty,tz,qx,qy,qz,qw;
        fin>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;

        Sophus::SE3d p1(Eigen::Quaterniond(qx,qy,qz,qw),Eigen::Vector3d(tx,ty,tz));
        trajectory.push_back(p1);
    }

    return trajectory;
}