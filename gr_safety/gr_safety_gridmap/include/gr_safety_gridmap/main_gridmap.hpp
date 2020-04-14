#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#ifndef MAINGRIDMAP_H
#define MAINGRIDMAP_H

namespace gr_safety_gridmap{
    class MainGrid{
        public:
            MainGrid(std::string iid="notmymap"): id(iid){
            };
            grid_map::GridMap gridmap;
            std::string id;
            boost::mutex mtx;
    };
};
static gr_safety_gridmap::MainGrid gridmap("thisismymap");
#endif
