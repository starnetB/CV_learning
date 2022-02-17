#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//简单类型的定义  
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//这是一个辅助教程，因此我们可以负担全局变量   
//创建可视化工具  
pcl::visualization::PCLVisualizer *p;
//定义左右视点  
int vp_1,vp_2;
//处理点云的方便的结构定义  
struct PCD
{
    PointCloud::Ptr cloud;
    std::string f_name;
    PCD():cloud(new PointCloud){};
};

struct PCDComparator
{
    bool operator()(const PCD& p1,const PCD& p2)
    {
        return (p1.f_name<p2.f_name);
    }
};

//以<x,y,z,curvature>形式定义一个新的点 
class MyPointRepresentation:public pcl::PointRepresentation<PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation()
    {
        //定义维度值
        nr_dimensions_=4;
    }

    //覆盖copyToFloatArray方法来定义我们的特征矢量 
    virtual void copyToFloatArray(const PointNormalT &p,float *out) const
    {
        //<x,y,z,curature> 
        out[0]=p.x;
        out[1]=p.y;
        out[2]=p.z;
        out[3]=p.curvature;
    }
};

/** 在可视化传沟的第一视点显示源点云和目标点云
 * 
 */
 
void showCloudLeft(const PointCloud::Ptr cloud_target,const PointCloud::Ptr cloud_source)
{
    //先清理一下  
    p->removePointCloud("vp1_target");
    p->removePointCloud("vp1_source");
    
    //标明点云的颜色  
    PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target,0,255,0);
    PointCloudColorHandlerCustom<PointT> src_h(cloud_source,255,0,0);

    //在视点1的角度取观看点云1和2
    p->addPointCloud(cloud_target,tgt_h,"vp1_target",vp_1);
    p->addPointCloud(cloud_source,src_h,"vp1_source",vp_1);

    PCL_INFO("Press q to begin the registration.\n");
    p->spin();
}

/** 在可视化窗口的第二视点显示源点云和目标点云，这边是以法向量的形式来实现的
 * 
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target,const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud("source");
    p->removePointCloud("target");

    //增加法向量字段 
    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target,"curvature");

    if(!tgt_color_handler.isCapable())
    {
        PCL_WARN("cannot create curvature color hanlder!");
    }
    
    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source,"curvature");
    if(!src_color_handler.isCapable())
        PCL_WARN("cannot create curvature color handler!");

    p->addPointCloud(cloud_target,tgt_color_handler,"target",vp_2);
    p->addPointCloud(cloud_source,src_color_handler,"source",vp_2);

    p->spinOnce();
}

/** 加载一组我们想要匹配在以其的PCD文件 
 * 参数argc是参数的数量(pass from main())
 * 参数argv实际的命令行参数（pass from main())
 * 参数models点云数据集的合成矢量 
 */

void loadData(int argc,char **argv,std::vector<PCD,Eigen::aligned_allocator<PCD>> &models)
{
    std::string extension(".pcd");
    //假定第一个参数是实际测试模型 
    for(int i=1;i<argc;i++)
    {
        std::string fname=std::string(argv[i]);
        //至少需要5个字符长(因为.plot就有5个字符)
        if(fname.size()<=extension.size())
            continue;
        std::transform(fname.begin(),fname.end(),fname.begin(),(int(*)(int))tolower);
        if(fname.compare(fname.size()-extension.size(),extension.size(),extension)==0)
        {
            //加载点云并保存在总体的模型列表中 
            PCD m;
            m.f_name=argv[i];
            pcl::io::loadPCDFile(argv[i],*m.cloud);
            //从点云中移除NAN点  
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud,indices);
            models.push_back(m);
        }
    }
}

/** 匹配一对点云数据集并且返还结果 
 * 参数 cloud_src 是源点云  
 * 参数 cloud_src 是目标点云 
 * 参数output输出的配准结果的源点云 
 * 参数final_transform是在来源和目标之间的转换 
 */

void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, 
               Eigen::Matrix4f &final_transform, bool downsample = false)
{
    //
	//为了一致性和高速的下采样
	//注意：为了大数据集需要允许这项
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);

    pcl::VoxelGrid<PointT> grid;
    if(downsample)
    {
        grid.setLeafSize(0.05,0.05,0.05);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);
        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }
    else{
        src=cloud_src;
        tgt=cloud_tgt;
    }

    //计算曲面法线条和曲率  这些是法向量的存储容器  
    PointCloudWithNormals::Ptr 	points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr  points_with_normals_tgt(new PointCloudWithNormals);

    //法向量计算容器  
    pcl::NormalEstimation<PointT,PointNormalT> norm_est;
    //创建kd树  
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    //计算src法向量 
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);
    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    //src中的xyz覆盖掉src_PN中的xyz的值，然后把xyz+normal的信息给src_PN  其中normal就是曲率
    //也就是没有法向量的值了，只有xyz的值加上 曲率的值，因为我们要使用，这些值来匹配点的特征
    pcl::copyPointCloud(*src,*points_with_normals_src);

    //计算tgt法向量  
    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt,*points_with_normals_tgt);
    //所以当copy的代码结束后，normals_tgt中就有x，y，z+normal的数据了

    //举例说明我们自定义点的表示（以上定义）
    //就是往我们的Input里面添加一个自定义的点，我们自己定义的点
	MyPointRepresentation point_representation;
	//调整'curvature'尺寸权重以便使它和x, y, z平衡
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//
    // 配准
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    //迭代最小参数 
	reg.setTransformationEpsilon(1e-6);
	//将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
	//注意：根据你的数据集大小来调整
    //如果两个点之间的距离越小就表示两者之间的距离更加接近，表示两者之间作为同一个点的对应关系越是精确，点对要求就更加严格
	reg.setMaxCorrespondenceDistance(0.1);
	//设置点表示
	reg.setPointRepresentation(std::make_shared<const MyPointRepresentation>(point_representation));
	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);
	//
	//在一个循环中运行相同的最优化并且使结果可视化
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    //每次计算迭代，内部优化次数
	reg.setMaximumIterations(2);

    //外部迭代，其实30*2一共迭代了60次
    for (int i = 0; i < 30; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);
		//为了可视化的目的保存点云
		points_with_normals_src = reg_result;
		//估计
		reg.setInputSource(points_with_normals_src);
        //每次都会重新更新reg_result,从而更新名义上的点云src的位置  
		reg.align(*reg_result);  //reg_result 称为输出目标
		//在每一个迭代之间累积转换
        //本次转换的矩阵，乘以上次的转换矩阵，默认出来的方向是H奇次矩阵是由目标指向源的方向（转换方向是sourcetotarget)
		Ti = reg.getFinalTransformation() * Ti;
		//如果这次转换和之前转换之间的差异小于阈值
		//则通过减小最大对应距离来改善程序
        //如果最后的转换的变换量，比上次转换的变换量小于阈值，就严格以下点对距离距离
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		prev = reg.getLastIncrementalTransformation();
		//可视化当前状态
		showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}
	//
	// 得到目标点云到源点云的变换(H方向是 source -> target) 因此求逆
	targetToSource = Ti.inverse();
	//
	//把目标点云转换回源框架
    //
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	p->removePointCloud("source");  //依据给定的ID，从屏幕中去除一个点云。參数是ID
	p->removePointCloud("target");
	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	p->addPointCloud(output, cloud_tgt_h, "target", vp_2);   //加入点云数据。下同 ,视点是vp_2
	p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	PCL_INFO("Press q to continue the registration.\n");
	p->spin();
	p->removePointCloud("source");
	p->removePointCloud("target");
	//添加源点云到转换目标
	*output += *cloud_src;
	final_transform = targetToSource;
}

//
 //****************  入口函数  ************************
 //主函数 

 int main (int argc, char** argv)
 {
    //读取数据 
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data; //模型
    loadData (argc, argv, data); //读取pcd文件数据，定义见上面

     //检查用户数据
    if (data.empty ())
    {
        PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]); //语法
        PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc"); //能够使用多个文件
        return (-1);
    }

    PCL_INFO ("Loaded %d datasets.", (int)data.size ()); //显示读取了多少个点云文件

    //创建一个 PCLVisualizer 对象
    p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example"); //p是全局变量
    p->createViewPort(0.0,0,0.5,1.0,vp_1);      //创建左视区
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2); //创建右视区

    //创建点云指针和变换矩阵
    PointCloud::Ptr result (new PointCloud), source, target; //创建3个点云指针，分别用于结果。源点云和目标点云
    //全局变换矩阵，单位矩阵。成对变换
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform; 


    //遍历全部点云文件
    for(size_t i=1;i<data.size();i++)
    {
        source = data[i-1].cloud; //源点云
        target = data[i].cloud; //目标点云
        showCloudLeft(source, target); //在左视区。简单的显示源点云和目标点云
        PointCloud::Ptr temp (new PointCloud); //创建暂时点云指针

        //显示正在配准的点云文件名称和各自的点数
        PCL_INFO ("Aligning %s (%d points) with %s (%d points).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        //********************************************************
        //配准2个点云。函数定义见上面
        pairAlign (source, target, temp, pairTransform, true);
        //将当前的一对点云数据，变换到全局变换中。(GlobalTrans  source -> target //targetTosource)
        pcl::transformPointCloud (*temp, *result, GlobalTransform);
        //更新全局变换 
        GlobalTransform=GlobalTransform*pairTransform;  //这个主要和source=data[i-1].cloud 
        //********************************************************

        // 保存成对的配准结果。变换到第一个点云帧
        std::stringstream ss; //这两句是生成文件名称

        pcl::io::savePCDFile (ss.str (), *result, true); //保存成对的配准结果

    }
 }