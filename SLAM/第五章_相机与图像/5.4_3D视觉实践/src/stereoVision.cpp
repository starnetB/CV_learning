#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

//文件路径 
string left_file="./left.png";
string right_file="./right.png";

//在pangolin中画图，已写好，无需调整
void showPointCloud(
    const vector<Vector4d,Eigen::aligned_allocator<Vector4d>> &pointcloud);

void showPointCloud(const vector<Vector4d,Eigen::aligned_allocator<Vector4d>> &pointcloud)
{
    if(pointcloud.empty()){
        cerr<<"Point cloud is empty"<<endl;
        return;
    }
    //create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Point Cloud Viewer",1024,768);
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
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    //定义投影和初始模型视图矩阵
    /**
     * ProjectionMatrix()
     * 设置相机内参，以及能看到的最大最小距离
     * 如ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000)
     * 表示相机分辨率，焦距， 相机光心，最小最大距离*/
    
    /**ModelViewLookAt()
     * 设置观看视角，pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
     * 的意思是在世界坐标(0，-0.1，-1.8)出观看坐标原点（0,0,0）并设置Y轴向上*/
    
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        //SetBounds 边界设置，最后一个是长宽比 -1024.0f/768.0f
        //前四个参数表示视图在视窗中的位置(这么写是在屏幕中央)
        .SetBounds(0.0,1.0,pangolin::Attach::Pix(175),1.0,-1024.0f/768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while(pangolin::ShouldQuit()==false){
         //先清空缓冲区
        glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

         //激活相机
        d_cam.Activate(s_cam);
        //清理颜色，也就是背景颜色
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p:pointcloud){
            glColor3f(p[3],p[3],p[3]);
            glVertex3d(p[0],p[1],p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);
    }
    return;
}
int main(int argc,char **argv){
    //内参
    double fx=718.856,fy=718.856,cx=607.1928,cy=185.2157;

    //基线
    double b=0.573;

    //读取图像
    cv::Mat left=cv::imread(left_file,0);
    cv::Mat right=cv::imread(right_file,0);
    /*  
    1、minDisparity 最小视差d。通常情况下，它是零，但有时整流算法可能会改变图像，所以这个参数需要作相应的调整。
    2、numDisparities 最大视差减去最小视察。该值总是大于零。在当前的实现中，该参数必须可以被16整除。
    3、BLOCKSIZE 匹配的块大小。它必须是> = 1的奇数。通常情况下，它应该在3…11的范围内，如何用周边的数来更新代价值。
    4、P1 控制视差平滑度的第一个参数。见下文。
    5、P2 第二个参数控制视差平滑度。值越大，差异越平滑。
         P1是相邻像素之间的视差变化加或减1的惩罚。
         P2是相邻像素之间的视差变化超过1的惩罚。该算法需要P2> P1。
         请参见stereo_match.cpp示例，
         其中显示了一些相当好的P1和P2值
        （ 分别为8 * number_of_image_channels * SADWindowSize * SADWindowSize和32 * number_of_image_channels * SADWindowSize * SADWindowSize）。
    6、disp12MaxDiff 左右视差检查中允许的最大差异（以整数像素为单位）。将其设置为非正值以禁用检查。 也就是允许d的最小值，也就是左右验证中的阈值
    7、preFilterCap 预滤波图像像素的截断值。该算法首先计算每个像素的x导数，并通过[preFilterCap，preFilterCap]间隔剪切其值。结果值传递给Birchfield-Tomasi像素代价函数。
    8、uniquenessRatio 最佳（最小）计算成本函数值应该“赢”第二个最佳值以考虑找到的匹配正确的百分比保证金。通常，5-15范围内的值就足够了。
    9、speckleWindowSize 平滑视差区域的最大尺寸，以考虑其噪声斑点和无效。将其设置为0可禁用斑点过滤。否则，将其设置在50-200的范围内。
    10、speckleRange 每个连接组件内的最大视差变化。如果你做斑点过滤，将参数设置为正值，它将被隐式乘以16.通常，1或2就足够好了。
    11、mode 将其设置为StereoSGBM :: MODE_HH以运行全尺寸双通道动态编程算法。它将消耗O（W * H * numDisparities）字节，这对640x480立体声很大，对于HD尺寸的图片很大。默认情况下，它被设置为false。
    */
    cv::Ptr<cv::StereoSGBM> sgbm=cv::StereoSGBM::create(0,96,9,8*9*9,32*9*9,1,63,10,100,32); //神奇的参数
    cv::Mat disparity_sgbm,disparity;
    //以下部分计算视差图,就是图中的每个点代表着，这个点在两个相机中的视差d，视差的最大值最好不要超过基线？有待验证
    sgbm->compute(left,right,disparity_sgbm);
    //CV_32F 像素在0-1.0f之间
    //1.0/16.0f
    /*
    m  目标矩阵。如果m的大小与原矩阵不一样，或者数据类型与参数不匹配，那么在函数convertTo内部会先给m重新分配空间。
    rtype 指定从原矩阵进行转换后的数据类型，即目标矩阵m的数据类型。当然，矩阵m的通道数应该与原矩阵一样的。如果rtype是负数，那么m矩阵的数据类型应该与原矩阵一样。
    alpha 缩放因子。默认值是1。即把原矩阵中的每一个元素都乘以alpha。
    beta 增量。默认值是0。即把原矩阵中的每一个元素都乘以alpha，再加上beta。*/
    //我的理解 输出图像在0-1536 之间，但是从上函数StereoSGBM::create 我们需要0-96.0之间的视差值，因此缩放1.0/16.0的比例
    disparity_sgbm.convertTo(disparity,CV_32F,1.0/16.0f);


    //生成点云
    vector<Vector4d,Eigen::aligned_allocator<Vector4d>> pointcloud;

    for(int v=0;v<left.rows;v++)
        for(int u=0;u<left.cols;u++){
            if(disparity.at<float>(v,u)<=0.0||disparity.at<float>(v,u)>96.0) 
                continue;  //防止视差值超出范围  //标定为原来的样子，不做任何修改
            Vector4d point(0, 0, 0, left.at<uchar>(v,u)/255.0);  //前三维为xyz,第四维为颜色

            // 根据双目模型计算point的位置
            double x=(u-cx)/fx;   //获得在相机坐标系下，x的值，x已经z轴归一化
            double y=(v-cy)/fy;    //获得在相机坐标系下，y的值，y已经z轴归一化
            double depth=fx*b/(disparity.at<float>(v,u));  //计算深度，根据视差图来计算
            point[0] =x*depth;
            point[1] =y*depth;       //反归一化
            point[2] =depth;

            pointcloud.push_back(point);

        }
    
    cv::imshow("disparity",disparity/96.0); //为了显示 彻底的归一化
    cv::waitKey(0);
    //画出点云
    showPointCloud(pointcloud);
    return 0;
}