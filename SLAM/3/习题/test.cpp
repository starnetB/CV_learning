#include<iostream>
#include<Eigen/Core>
#include<Eigen/Geometry>

using namespace Eigen;
using namespace std;

int main()
{
    Matrix3d IM;
    Isometry3d T=Isometry3d::Identity();
    IM=T.matrix().block<3,3>(0,0);
    cout<<IM<<endl;

}