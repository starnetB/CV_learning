#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

// Path to trajector file
string trajectory_file ="trajectory.txt";

void DrawTrajectory(vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>>);

int main(int argc,char **argv)
{
    vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>> poses;
    
    ifstream fin(trajectory_file);
    if(!fin){
        cout<<"can not find trajectory file at "<<trajectory_file<<endl;
        return -1;
    }
    while(!fin.eof()){
        double time,tx,ty,tz,qx,qy,qz,qw;
        fin>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Isometry3d Twr(Quaterniond(qw,qx,qy,qz));
        Twr.pretranslate(Vector3d(tx,ty,tz));
        poses.push_back(Twr);
    }
    cout<<"read total"<<poses.size()<<"pose entries"<<endl;

    //draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

void DrawTrajectory(vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>> poses)
{
    //create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer",1024,768);
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
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC1_ALPHA);

    //定义投影和初始模型视图矩阵
    /**
     * ProjectionMatrix()
     * 设置相机内参，以及能看到的最大最小距离
     * 如ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000)
     * 表示相机分辨率，焦距，相机光心，最小最大距离*/

    /**ModelViewLookAt()
     * 设置观看视角，pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
     * 的意思是在世界坐标(0，-0.1，-1.8)出观看坐标原点（0,0,0）并设置Y轴向上*/

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
        pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1.0,0.0)
    );

    pangolin::View &d_cam=pangolin::CreateDisplay()
        //SetBounds 边界设置，最后一个是长宽比 -1024.0f/768.0f
        //前四个参数表示视图在视窗中的位置(这么写是在屏幕中央)
        .SetBounds(0.0,1.0,0.0,1.0,-1024.0f/768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));
    
    while(pangolin::ShouldQuit()==false){
        //先清空缓冲区
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        //激活相机
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f); //清理颜色，也就是背景颜色
        glLineWidth(2);
        for(size_t i=0;i<poses.size();i++)
        {
            //画每个位姿的三个坐标轴
            Vector3d Ow=poses[i].translation(); //坐标原点
            Vector3d Xw=poses[i]*(0.1*Vector3d(1,0,0));  //x轴
            Vector3d Yw=poses[i]*(0.1*Vector3d(0,1,0));  //y轴
            Vector3d Zw=poses[i]*(0,1*Vector3d(0,0,1));  //z轴
            /**
             * 画线: (给两点,自动连线)
             * glLineWidth(3);//线宽
             * glBegin (GL_LINES);//开始画线
             * glColor3f ( 0.8f,0.f,0.f );//颜色
             * glVertex3f( -1,-1,-1 );//线起点
             * glVertex3f( 0,-1,-1 );//线终点
             * glEnd();//结束画线*/
            glBegin(GL_LINES);
            glColor3f(1.0,0.0,0.0);
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            glVertex3d(Xw[0],Xw[1],Xw[2]);

            glColor3f(0.0,1.0,0.0);
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            glVertex3d(Yw[0],Yw[1],Yw[2]);

            glColor3f(0.0,0.0,1.0);
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            glVertex3d(Zw[0],Zw[1],Zw[2]);

            glEnd();
        }

        // 画出连线
        for(size_t i=0;i<poses.size();i++)
        {
            glColor3f(0.0,0.0,0.0);
            glBegin(GL_LINE);
            auto p1=poses[i],p2=poses[i+1];
            glVertex3d(p1.translation()[0],p1.translation()[1],p1.translation()[2]);
            glVertex3d(p2.translation()[0],p2.translation()[1],p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000); //sleep 5 ms
    }
}