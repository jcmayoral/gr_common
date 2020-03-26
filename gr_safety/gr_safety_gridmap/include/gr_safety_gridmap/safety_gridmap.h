#include <ros/ros.h>
#include <gr_safety_gridmap/layer_subscriber.h>
#include <nav_msgs/Path.h>

namespace gr_safety_gridmap{
    class SafetyGridMap{
        public:
           SafetyGridMap();
           void Update();
        private:
            ros::Subscriber sub_;
            ros::Publisher pub_;
            std::map<std::string, LayerSubscriber<int>> layer_subscribers;
    };
}

/*
ros::AdvertiseOptions createAdvertiseOptions(const ConnectionInfo* c, uint32_t queue_size, const std::string& prefix) {
    ros::AdvertiseOptions opts(prefix + c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    opts.latch = isLatching(c);
    return opts;
}
*/
