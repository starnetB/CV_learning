#include <vector>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>
#include <opencv2/imgcodecs/legacy/constants_c.h>

using namespace std;
using namespace cv;


void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);

Point2d pixel2cam(const Point2d &p,const Mat &K);

//BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose
);

void bundleAdjustmentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose
);

int main(int argc, char **argv) {
    if (argc != 5) {
        cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 建立3D点
    Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for (DMatch m:matches) {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)   // bad depth
        continue;
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }

    cout << "3d-2d pairs: " << pts_3d.size() << endl;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;

    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;

    //填充3d模型 
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); ++i) {
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }

    cout<<"calling bundle adjustment by guass newton"<<endl;
    Sophus::SE3d pose_gn;
    t1 = chrono::steady_clock::now();
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << endl;

    cout << "calling bundle adjustment by g2o" << endl;
    Sophus::SE3d pose_g2o;
    t1 = chrono::steady_clock::now();
    bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;
    return 0;
}

void bundleAjustmentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose
){
    typedef Eigen::Matrix<double,6,1> Vector6d;
    const int iterations=10;
    double cost=0,lastCost=0;
    double fx=K.at<double>(0,0);
    double fy=K.at<double>(0,1);
    double cx=K.at<double>(0,2);
    double cy=K.at<double>(1,2);

    for (int iter=0;iter<iterations;iter++){
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost=0;
        //compute cost 
        for (int i=0;i<points_3d.size();i++){
            Eigen::Vector3d pc=pose*points_3d[i]; //这边成功获取3维坐标下的点
            
            double inv_z=1.0/pc[2];
            double inv_z2=inv_z*inv_z;

            //获取像素预测值
            Eigen::Vector2d proj(fx*pc[0]/pc[2]+cx,fy*pc[1]/pc[2]+cy);
            
            //获取当前误差选项  
            Eigen::Vector2d e=points_2d[i]-proj;

            cost+=e.squaredNorm();  //叠加损失函数

            Eigen::Matrix<double,2,6> J;
            J<<-fx*inv_z,
                0,
                fx*pc[0]*inv_z2,
                fx*pc[0]*pc[1]*inv_z2,
                -fx-fx*pc[0]*pc[0]*inv_z2,
                fx*pc[1]*inv_z,
                0,
                -fy * inv_z,
                fy * pc[1] * inv_z2,
                fy + fy * pc[1] * pc[1] * inv_z2,
                -fy * pc[0] * pc[1] * inv_z2,
                -fy * pc[0] * inv_z;

            H+=J.transpose()*J;   //牛顿公式中的H
            b+=-J.transpose()*e;  //牛顿公式中的b(gx)
        }
        Vector6d dx;
        dx=H.ldlt().solve(b);

        if(isnan(dx[0])){
            cout<<"result is nan!"<<endl;
            break;
        }
        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << endl;
        if (dx.norm() < 1e-6) {
            // converge
            break;
        }
    }
    cout<<"pose by g-n:\n"<<pose.matrix()<<endl;
}

///vertex and edges used in g2o ba 
class VertexPose:public g2o::BaseVertex<6,Sophus::SE3d>{
    public:   //这里的6指明了节点的维度6 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {//初始化一个节点
        _estimate=Sophus::SE3d();
    }

    ///left multiplication on SE3  
    //定义更新规则 
    virtual void oplusImpl(const double *update) override{
        Eigen::Matrix<double,6,1> update_eigen;
        update_eigen <<update[0],update[1],update[2],update[3],update[4],update[5];
        _estimate=Sophus::SE3d::exp(update_eigen)*_estimate;
    }

    virtual bool read(istream &in) override{}

    virtual bool write(ostream &out) const override{}
};


//下面开始定义误差 ，也就是边
//BaseUnaryEdge   一元边
//BaseBinaryEdge  二元边
//BaseMultiEdege  多元边
//<2 ,Eigen::Vector2d,VectexPose> 
//2 被动，被测量对象的维度，这里的话，是指像素点
//Eigen::Vector<2d> 被测量对象类型
//VertexPose 节点
class EdegeProjection:public g2o::BaseUnaryEdge<2,Eigen::Vector2d,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdegeProjection(const Eigen::Vector3d &pos,
                    const Eigen::Matrix3d &K):_pos3d(pos),_K(K){}
    
    virtual void computeError() override{
        const VertexPose *v=static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T=v->estimate();
        Eigen::Vector3d pos_pixel= _K*(T*_pos3d);
        pos_pixel/=pos_pixel[2];
        _error=_measurement-pos_pixel.head<2>();  //这里其实要设定b
        // 像素来自于外面的_measurement
        // 三维点可以制定
        // 最终获得error，用于迭代
    }

    virtual void linearizeOplus() override{
        const VertexPose *v=static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T=v->estimate();
        Eigen::Vector3d pos_cam=T*_pos3d;
        double fx=_K(0,0);
        double fy=_K(1,1);
        double cx=_K(0,2);
        double cy=_K(1,2);
        double X=pos_cam(0);
        double Y=pos_cam(1);
        double Z=pos_cam(2);
        double Z2=Z*Z;
        _jacobianOplusXi
        << -fx/Z,0,fx*X/Z2,fx*X*Y/Z2,-fx-fx*X*X/Z2,fx*Y/Z,
           0,-fy/Z,fy*Y/Z2,fy+fy*Y*Y/Z2,-fy*X*Y/Z2,-fy*X/Z;
    }

    virtual bool read(istream &in) override{}

    virtual bool write(ostream &out) const override{}

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};

//上面定义完 边和节点，我们可以进行优化了  

void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose
){
    //构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> BlockSolverType;
    //pose is 6,landmark is 3 也就是每次迭代T是6维度的李代数，路标就是世界坐标系下的 X《Y，Z
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    // 线性求解器
    //梯度下降方法，可以从GN，LM，DolLeg中选择 
    auto solver=new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    
    g2o::SparseOptimizer optimizer;  //图模型
    optimizer.setAlgorithm(solver); //设置求解其
    optimizer.setVerbose(true); //打开调试输出


    //vertex
    //设定一个初始化节点
    VertexPose *vertex_pose=new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);

    //K  加载相机内参数

    Eigen::Matrix3d K_eigen;
    K_eigen <<
        K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // edges 加载边和其他节点   
    int index=1;
    for (size_t i=0;i<points_2d.size();++i)
    {
        auto p2d=points_2d[i];
        auto p3d=points_3d[i];
        EdegeProjection *edge=new EdegeProjection(p3d,K_eigen);
        edge->setId(index);
        edge->setVertex(0,vertex_pose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl;
    pose = vertex_pose->estimate();
};