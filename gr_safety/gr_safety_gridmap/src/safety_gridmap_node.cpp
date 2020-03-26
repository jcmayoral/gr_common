#include <gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

int main(int argc, char** argv){
    ros::init(argc,argv,"gr_safety_gridmap_node");
    SafetyGridMap safety_gridmap;
    ros::spin();
}