#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc,char** argv){
    //Quaterniond q1(0.35,0.2,0.3,0.1),q2(-0.5,0.4,-0.1,0.2);
    //实验作者表述有误
    Quaterniond q1(1,0,0,0),q2(1,0,0,0);
    q1.normalize();
    q2.normalize();
    //Vector3d t1(0.3,0.1,0.1),t2(-0.1,0.5,0.3);
    //Vector3d p1(0.5,0,0.2);
    Vector3d t1(0.5,0.5,0.0),t2(0,0,0);
    Vector3d p1(0.5,0.5,0.0);

    Isometry3d T1w(q1),T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    cout<<T2w.matrix()<<endl;

    //作者的理解是相反的？
    Vector3d p2=T2w*T1w.inverse()*p1;
    cout<<endl<<p2.transpose()<<endl;

    //这段表述有误，正确的应该如下所示

    Vector3d p3=T2w.inverse()*T1w*p1;
    cout<<endl<<p3.transpose()<<endl;

    return 0;
}