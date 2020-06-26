#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#ifndef MAINGRIDMAP_H
#define MAINGRIDMAP_H

namespace gr_safety_gridmap{
    class MainGrid{
        public:
            MainGrid(std::string iid="notmymap"): id(iid), is_new_data_{false}{
            };
            void setDataFlag(bool is_ready){
                is_new_data_ = is_ready;
            }
            bool isNewDataAvailable(){
                return is_new_data_;
            }
            grid_map::GridMap gridmap;
            std::string id;
            bool is_new_data_;
            boost::mutex mtx;
            std::map<std::string, ros::Time> update_times_;
    };
};
static gr_safety_gridmap::MainGrid gridmap("thisismymap");
#endif
