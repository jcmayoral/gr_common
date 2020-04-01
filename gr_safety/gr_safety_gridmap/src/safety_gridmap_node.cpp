#include <gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

int main(int argc, char** argv){
    ros::init(argc,argv,"gr_safety_gridmap_node");
    SafetyGridMap safety_gridmap;

    ros::Rate r(10);

    while(ros::ok()){
        //TODO replace by rate
        //ros::Duration(0.5).sleep();
        safety_gridmap.publishGrid();
        r.sleep();
        safety_gridmap.updateGrid();
        ros::spinOnce();
    }
    return 0;
}