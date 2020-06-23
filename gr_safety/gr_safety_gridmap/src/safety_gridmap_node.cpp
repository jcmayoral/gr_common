#include <gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

int main(int argc, char** argv){

    ros::init(argc,argv,"gr_safety_gridmap_node");
    SafetyGridMap *safety_gridmap;
    std::string localstr(argv[1]);
    bool localgr = true;

    if (localstr.compare("global") == 0){
        localgr = false;
    }

    safety_gridmap = new SafetyGridMap(localgr);

    ros::Rate r(20);

    while(ros::ok()){
        //TODO replace by rate
        //ros::Duration(0.5).sleep();
        //<Wsafety_gridmap->publishGrid();
        r.sleep();
        safety_gridmap->updateGrid();
        ros::spinOnce();
    }
    return 0;
}
