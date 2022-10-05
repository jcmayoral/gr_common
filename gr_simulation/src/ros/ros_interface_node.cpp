#include <ros/ros_interface.h>

int main(int argc, char** argv){
    std::string action_id = "SimMotionPlanner/my_person";
    std::string node_name = "interface_for_person";
    if (argc>1){
        std::cout <<argv[1] << std::endl;
        action_id += argv[1];
        node_name += argv[1];
    }
    ros::init(argc,argv, node_name);
    gr_simulation::ROSInterface interface(action_id);
    return 1;
}
