#include <gr_safe_gridmap/safe_gridmap.h>

using namespace gr_safe_gridmap;

int main(int argc, char** argv){

    ros::init(argc,argv,"gr_safe_gridmap_node");
    SafeGridMap *safe_gridmap;
    std::string localstr(argv[1]);
    bool localgr = true;

    if (localstr.compare("global") == 0){
        localgr = false;
    }

    safe_gridmap = new SafeGridMap(localgr);

    ros::Duration r(0.10);

    while(ros::ok()){
        //TODO replace by rate
        //ros::Duration(0.5).sleep();
        //safe_gridmap->publishGrid();
        r.sleep();
        safe_gridmap->updateGrid();
        ros::spinOnce();
    }
    return 0;
}
