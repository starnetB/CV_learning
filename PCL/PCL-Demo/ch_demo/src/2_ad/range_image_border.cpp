#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

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
        printUsage(argv[0]);
        return 0;
    }
    if(pcl::console::find_argument(argc,argv,"-m")>=0)
    {
        setUnseenToMaxRange=true;
        cout<<"Setting unseen values in range image to maximum range readings.\n";
    }
    int tmp_coordiante_frame;
    //if(pcl::console::parse)
}