#include <gazebo/obstacle_coordinator.h>

int main (int argc, char** argv){
    gazebo::client::setup(argc, argv);
    gazebo::ObstaclesCoordinator coordinator(std::atoi(argv[1]), std::atof(argv[2]));
    coordinator.start();

    gazebo::client::shutdown();
    return 1;
}