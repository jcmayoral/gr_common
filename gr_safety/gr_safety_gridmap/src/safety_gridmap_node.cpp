#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

using namespace grid_map;

int main(int argc, char** argv){
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_simple_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    // Create grid map.
    GridMap map({""});
    //map.add("elevation", 2.0 * Matrix::Random(map.getSize()(0), map.getSize()(1)));
    //map.add("noise", Matrix::Random(map.getSize()(0), map.getSize()(1)));
    map.add("normal_x", Matrix::Random(map.getSize()(0), map.getSize()(1)));
    map.add("normal_y", Matrix::Random(map.getSize()(0), map.getSize()(1)));
    map.add("normal_z", Matrix::Random(map.getSize()(0), map.getSize()(1)));

    map.setFrameId("map");
    map.setGeometry(Length(2.0, 2.0), 0.05);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1));
    // Work with grid map in a loop.
    ros::Rate rate(30.0);

    while (nh.ok()) {

    // Add data to grid map.
    ros::Time time = ros::Time::now();
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        //map.at("noise", *it) = rand();
        //map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
        Eigen::Vector3d normal(-0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()),
                             -position.x() * std::cos(3.0 * time.toSec() + 5.0 * position.y()), 1.0);
        normal.normalize();
        map.at("normal_x", *it) = normal.x();
        map.at("normal_y", *it) = random() % 5 + -5 ;//normal.y();
        map.at("normal_z", *it) = -1;//normal.z();
    }

    map.add("sum", map.get("normal_x") + map["normal_y"] + map["normal_z"]);
    map.add("product", map.get("normal_z") - map["normal_y"]);

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
        message.info.header.stamp.toSec());
    // Wait for next cycle.
    rate.sleep();
    }
    return 0;
}