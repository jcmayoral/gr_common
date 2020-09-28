#include <ros/ros_interface.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"interface_for_person");
    gr_simulation::ROSInterface interface;
    return 1;
}
