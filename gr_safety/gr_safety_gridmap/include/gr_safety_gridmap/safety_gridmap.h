#include <ros/ros.h>
#include <ros/package.h>
#include <gr_safety_gridmap/layer_subscriber.hpp>
//#include <gr_safety_gridmap/main_gridmap.hpp>
#include <yaml-cpp/yaml.h>

namespace gr_safety_gridmap{
    class SafetyGridMap{
        public:
           SafetyGridMap();
           SafetyGridMap(bool localgridmap);
           void initializeGridMap(bool localgridmap);
           void updateGrid();
           void publishGrid();
           void addStaticLayer(std::string iid);
        private:
            ros::Publisher rpub_;
            std::vector<LayerSubscriber> layer_subscribers;
    };

};

/*
ros::AdvertiseOptions createAdvertiseOptions(const ConnectionInfo* c, uint32_t queue_size, const std::string& prefix) {
    ros::AdvertiseOptions opts(prefix + c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    opts.latch = isLatching(c);
    return opts;
}
*/
