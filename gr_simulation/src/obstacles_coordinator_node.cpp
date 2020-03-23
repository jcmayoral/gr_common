#include <obstacle_coordinator.h>

int main (int argc, char** argv){
    gazebo::client::setup(argc, argv);
    std::cout << "a ";
    gazebo::ObstaclesCoordinator coordinator(std::atoi(argv[1]));
    std::cout << "a ";
    coordinator.start();
    std::cout << "a ";
    while(true);

    gazebo::client::shutdown();
    return 1;
}