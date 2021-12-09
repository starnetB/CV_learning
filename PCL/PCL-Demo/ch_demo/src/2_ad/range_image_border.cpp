#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h>

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame=pcl::RangeImage::LASER_FRAME;
bool setUnseenToMaxRange=false;

// --------------
// -----Help-----
// --------------

void printUsage(const char* progName)
{
     std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
              << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
              << "-m           Treat all unseen points to max range\n"
              << "-h           this help\n"
              << "\n\n";
}
// --------------
// -----Main-----
// --------------

int main(int argc,char **argv)
{
    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if(pcl::console::find_argument(argc,argv,"-h")>=0)
    {
        //如果输入是-h，那么提供帮助信息
        printUsage(argv[0]);
        return 0;
    }
    if(pcl::console::find_argument(argc,argv,"-m")>=0)
    {
        //对待那些理应被观察到，但超出范围的点使用最大化深度的方式
        setUnseenToMaxRange=true;
        cout<<"Setting unseen values in range image to maximum range readings.\n";
    }
    int tmp_coordiante_frame;
    if(pcl::console::parse(argc,argv,"-c",tmp_coordiante_frame)>=0)
    {
        coordinate_frame=pcl::RangeImage::CoordinateFrame (tmp_coordiante_frame);
        cout<<"Using coordinate frame "<<(int)coordinate_frame<<" .\n";
    }
    if(pcl::console::parse(argc,argv,"-r",angular_resolution)>=0)
    {
        cout<<"Setting angular resolution to "<<angular_resolution<<"deg.\n";
    }
    angular_resolution=pcl::deg2rad(angular_resolution);  //角度变成弧度

    // ------------------------------------------------------------------
    // -----Read pcd file or create example point cloud if not given-----
    // ------------------------------------------------------------------
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& point_cloud=*point_cloud_ptr;
    
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    // 看看命令行里面有没有带有pcd的文件
    std::vector<int> pcd_filename_indices=pcl::console::parse_file_extension_argument(argc,argv,"pcd");
    if(!pcd_filename_indices.empty())
    {
        std::string filename=argv[pcd_filename_indices[0]];  //如果有的花把第一个文件给拿出来
        if(pcl::io::loadPCDFile(filename,point_cloud)==-1)
        {
            cout<<" Was not able to open file \""<<filename<<"\".\n";  //打开文件失败
        }

        //从点云文件中获取场景传感器的位置
        scene_sensor_pose=Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2]))
                                        * Eigen::Affine3f(point_cloud.sensor_orientation_);
        //重新命名文件系统
        std::string far_ranges_filename=pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
        if(pcl::io::loadPCDFile(far_ranges_filename.c_str(),far_ranges)==-1)
        {
            //使用家在深度范围文件
            cout<<"Far ranges file \""<<far_ranges_filename<<"\" does not existes does not exists.\n";
        }
    }
    else //如果没有点云文件，我们就要自己出点云数据了
    {
        cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
        for (float x=-0.5f; x<=0.5f; x+=0.01f)
        {
            for (float y=-0.5f; y<=0.5f; y+=0.01f)
            {
                PointType point;
                point.x=x;
                point.y=y;
                point.z=2.0f-y;
                point_cloud.points.push_back(point);
            }
        }
        point_cloud.width  = (int)point_cloud.points.size();
        point_cloud.height = 1;
    }
    
        // -----------------------------------------------
        // -----Create RangeImage from the PointCloud-----
        // -----------------------------------------------
        float noise_level = 0.0;
        float min_range = 0.0f;
        int border_size = 1;
        boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
        pcl::RangeImage& range_image = *range_image_ptr;
        //这里的scene_sensor_pose ,如果无法从pcd文件的点云中获得，那么我们就是用默认的初始位置
        range_image.createFromPointCloud(point_cloud,angular_resolution,
                                         pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                         scene_sensor_pose,coordinate_frame,
                                         noise_level,min_range,border_size);
        //将深度范围文件加载到range_image中区，限定范围
        //far_ranges 提供了超出范围 不可见点，这些对于轮廓提取有非常的重要
        range_image.integrateFarRanges(far_ranges);
        if(setUnseenToMaxRange)
        {
            //如果无法提供超出这些应该观察到的传感器范围的点，则可以使用setUnseenToMaxRange函数，将那些点设置为最大深度（本例添加-m参数）。
            range_image.setUnseenToMaxRange();
        }

        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.setBackgroundColor(1,1,1);
        viewer.addCoordinateSystem(1.0f,"global");
        pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr,0,0,0);
        viewer.addPointCloud(point_cloud_ptr,point_cloud_color_handler,"original_point_cloud");
        //PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
        //viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
        //viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");

        // -------------------------
        // -----Extract borders-----
        // -------------------------
        //这里教你怎么用之前的轮廓图来提取边界
        //抽取边界
        pcl::RangeImageBorderExtractor border_extractor(&range_image);
        pcl::PointCloud<pcl::BorderDescription> border_descriptions;
        //加载边界描述
        border_extractor.compute(border_descriptions);
        //要提取的框框有以下几点
        //border_points_ptr 对象边框
        //veil_points_ptr   对象与阴影之间的点
        //shadow_points_ptr 阴影点
         pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
        pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
            & veil_points = * veil_points_ptr,
            & shadow_points = *shadow_points_ptr;
        for (int y=0;y<(int)range_image.height;++y)
        {
            for(int x=0;x<(int)range_image.width;++x)
            {   
                //如果某一个点是障碍物边界点，那么将他放到border_points里面
                if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                    border_points.points.push_back (range_image.points[y*range_image.width + x]);
                //如果某一个点是veil点，那么将他放到veil_points.里面
                if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                    veil_points.points.push_back (range_image.points[y*range_image.width + x]);
                //如果某一个点是shadow点，那么将他放到shadow_points里面
                if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                    shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
            }
        }
    //分别对三种类型的点进行可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
    viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
    ////-------------------------------------
    // -----Show points on range image-----
    // ------------------------------------
    //用深度图展示可视化内容
    pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL; //初始化一个窗口
    range_image_borders_widget =
            pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, 
            -std::numeric_limits<float>::infinity (),
             std::numeric_limits<float>::infinity (), false,
            border_descriptions, "Range image with borders");
     // -------------------------------------                              


    //--------------------
    // -----Main loop-----
    while(!viewer.wasStopped())
    {
        range_image_borders_widget->spinOnce();
        viewer.spinOnce();
        pcl_sleep(0.01);
    }                                                                                     
}