#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// 首先从定义类型开始，以免使代码混乱
// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;  //FPFH 特征描述子 
typedef typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;   //这个应该是FPFH的描述子获取器  
typedef pcl::PointCloud<FeatureT> FeatureCloudT; //和点云相对应的特征描述子伪点云,里面存储着一个个点云描述子
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT; //颜色处理器,显示的时候使用  

int main(int argc, char **argv)
{
     // 然后，我们实例化必要的数据容器，检查输入参数并加载对象和场景点云
     // 这里我们解释下
     // inputCloud:是要被转换的对象
     // inputTarget:就是参考系,人家要往他那边转换
     // Point clouds
    PointCloudT::Ptr object (new PointCloudT);
     // 要输出的已经被对奇的对象目标  
    PointCloudT::Ptr object_aligned (new PointCloudT);
     // 场景点云  
    PointCloudT::Ptr scene (new PointCloudT);
     
     //特征点云
     //对象特征点云 
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    // 场景特征点云
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);
    // Get input object and scene
    if (argc != 3)
    {
        pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
        return (1);
    }
    // Load object and scene
    // 高亮输出一段话 
    pcl::console::print_highlight ("Loading point clouds...\n");
    if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
    {
        pcl::console::print_error ("Error loading object/scene file!\n");
        return (1);
    }

    // Downsample
    // 为了加快处理速度，我们使用PCL的：pcl::VoxelGrid类将对象和场景点云的采样率下采样至5 mm。
    pcl::console::print_highlight ("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (object);
    grid.filter (*object);  //输出过滤目标
    grid.setInputCloud (scene);
    grid.filter (*scene);//输出场景过滤目标

    // Estimate normals for scene
    // 使用OMP重新计算场景法向量 
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    nest.setRadiusSearch (0.01);
    nest.setInputCloud (scene);
    nest.compute (*scene);

    // Estimate features
    // 对于下采样点云中的每个点，我们现在使用PCL的pcl::FPFHEstimationOMP<>类来计算用于对齐过程中用于匹配的快速点特征直方图（FPFH）描述符。
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (0.025);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    fest.setInputCloud (scene);
    fest.setInputNormals (scene);
    fest.compute (*scene_features);

    // Perform alignment
    // SampleConsensusPrerejective 实现了有效的RANSAC姿势估计循环
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
    align.setInputSource (object);
    //设置一个指向源点云的特征描述对象的 boost 库的共享指针 features.
    align.setSourceFeatures (object_features);
    
    align.setInputTarget (scene);
    //设置一个指向目标点云的特征描述对象的 boost 库的共享指针 features.
    align.setTargetFeatures (scene_features);

    //最大迭代次数
    align.setMaximumIterations (50000); // Number of RANSAC iterations

    //设置每次迭代中使用的样本数量，为了正常估计至少需要3对。
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose

    //设置计算协方差时选择附近多少个点为邻居 ：设置的点的数量 k 越高，协方差计算越准确但也会相应降低计算速度
    align.setCorrespondenceRandomness (5); // Number of nearest features to use

    //设置多边形相似阈值，对于目标物体和场景上的对应点，根据他们在各自的空间里之间的欧式距离是不变得这一几何特性，使用类CorrespondenceRejectorPoly去消除错误对应点；这个值越接近1 ，该算法通过减少迭代次数而变得月快速；但是，当噪声存在时，这也增加了排查正确对应点对的风险。
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);
    }

    if (align.hasConverged ())
    {
        // Print results
        printf ("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

        // Show alignment
        pcl::visualization::PCLVisualizer visu("Alignment");
        visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        visu.spin ();
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
        return (1);
    }

    return (0);
}