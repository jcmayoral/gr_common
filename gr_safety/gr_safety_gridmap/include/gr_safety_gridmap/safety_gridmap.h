#include <ros/ros.h>
#include <gr_safety_gridmap/layer_subscriber.hpp>
#include <nav_msgs/Path.h>
#include <memory>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace gr_safety_gridmap{
    class SafetyGridMap{
        public:
           SafetyGridMap();
           void updateGrid();
           void publishGrid();
           grid_map::GridMap cmap_;
        private:
            ros::Publisher rpub_;
            std::map<std::string, LayerSubscriber<nav_msgs::Path>> layer_subscribers;
    };


};

/*
ros::AdvertiseOptions createAdvertiseOptions(const ConnectionInfo* c, uint32_t queue_size, const std::string& prefix) {
    ros::AdvertiseOptions opts(prefix + c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    opts.latch = isLatching(c);
    return opts;
}
*/
