#include <obstacle_coordinator.h>

int main (int argc, char** argv){
    gazebo::client::setup(argc, argv);
    gazebo::ObstaclesCoordinator coordinator(std::atoi(argv[1]));
    coordinator.start();
    while(true);

    gazebo::client::shutdown();
    return 1;
}