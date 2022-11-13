#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;

//曲线模型的顶点，模板参数：优化变量的维度和数据类型
class CurveFittingVertex:public g2o::BaseVertex<3,Eigen::Vector3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //重置，abc的起始变量值是0，0，0？
    virtual void setToOriginImpl() override{
        _estimate<<0,0,0;
    }

    //更新 规则  用更性的三维数组去更新 顶点
    virtual void oplusImpl(const double *update) override{
        _estimate += Eigen::Vector3d(update);
    }

    //存盘和读盘 ：预留
    virtual bool read(istream &in){return true;}
    virtual bool write(ostream &out) const{return true;}
};

//误差模型 模板参数：观测值维度，类型，连接顶点类型,边
//一元边
class CurveFittingEdge:public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x):BaseUnaryEdge(),_x(x){}


    //计算曲线的误差模型、
    virtual void computeError() override{
        const CurveFittingVertex *v=static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc=v->estimate();
        _error(0,0) = _measurement-std::exp(abc(0,0)*_x*_x+abc(1,0)*_x+abc(2,0));
    }

    //计算雅可比矩阵
    virtual void linearizeOplus()  override{
        const CurveFittingVertex *v=static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc=v->estimate();
        double y=exp(abc[0]*_x*_x+abc[1]*_x+abc[2]);
        _jacobianOplusXi[0]=-_x*_x*y;
        _jacobianOplusXi[1]=-_x*y;
        _jacobianOplusXi[2]=-y;
    }

    virtual bool read(istream &in){return true;}
    virtual bool write(ostream &out) const {return true;}  //1.当成员函数后面加const，函数中所有类中的成员属性不能被改变
public:
    double _x;  //x值，

};

int main(int argc, char **argv){
    double ar=1.0,br=2.0,cr=1.0;      //真实参数值
    double ae=2.0,be=-1.0,ce=5.0;     //估计参数值
    int N=100;                        //数据点数量
    double w_sigma=1.0;               //噪声Sigma值
    double inv_sigma=1.0/w_sigma;     
    cv::RNG rng;                      //OpenCV随机数产生器

    vector<double> x_data,y_data;  //数据
    for(int i=0;i<N;i++){
        double x=i/100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar*x*x+br*x+cr)+rng.gaussian(w_sigma*w_sigma));
    }

    double abc[3]={ae,be,ce};

    //求解块，指定求解块，参数的维度
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> BlockSolverType;  //每个误差项优化变量维度为3，误差值维度为1

    //指定，求解器类型
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; //线性求解器类型

    //梯度下降方法，可以从GN，LM，DogLeg中选
    auto solver=new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;   //图模型
    optimizer.setAlgorithm(solver);   //设置求解器
    optimizer.setVerbose(true);       //打开调试输出
    

    //向图里面增加顶点
    CurveFittingVertex *v=new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae,be,ce));
    v->setId(0);   //设定顶点的初始Id
    optimizer.addVertex(v);

    //往图中增加边
    for(int i=0;i<N;i++){
        CurveFittingEdge *edge=new CurveFittingEdge(x_data[i]);
        edge->setId(i);                        //设置边的id
        edge->setVertex(0,v);                  //设置顶点0，id的顶点指向 V，指向同一个顶点
        edge->setMeasurement(y_data[i]);   //观测数据
        edge->setInformation(Eigen::Matrix<double,1,1>::
                            Identity()*1/(w_sigma*w_sigma));
                             //信息矩阵，协方差矩阵之逆，
                             //这里 使用的是一维度的协方差矩阵
                             //其实这里的协方差矩阵真心没啥用，但要设置
                             //求目标函数的时候是有用的
        optimizer.addEdge(edge);
    }

    //开始执行优化项目
    cout<<"start optimization"<<endl;
    chrono::steady_clock::time_point t1=chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2=chrono::steady_clock::now();
    chrono::duration<double> time_used=chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"solve time cost =  "<< time_used.count() << "seconds. " <<endl;

    //输出优化值
    Eigen::Vector3d abc_estimate=v->estimate();
    cout<<"estimate model:  "<<abc_estimate.transpose()<<endl;

    return 0;
}

//这里总结下，这里点只是在意优化变量的维度，而不会在意具体的观测量，通过setEstimate(Eigen::Vector3d(ae,be,ce))将优化变量传入
//边需要传入，常数参数x和观测值y(setMeasurement(y_data[i]);)，并通过点来待优化变量，当然可以传入信息矩阵