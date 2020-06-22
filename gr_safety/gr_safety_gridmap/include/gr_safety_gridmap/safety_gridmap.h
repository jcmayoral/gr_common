#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
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
           void loadRegions(std::string iid);
           void timer_callback(const ros::TimerEvent& event);
        private:
            ros::Publisher rpub_;
            ros::Publisher safety_grader_;
            std::vector<LayerSubscriber> layer_subscribers;
            ros::Timer clear_timer_;
            boost::mutex smtx;
            std::map<std::string, ros::Time> update_times_;
    };

};
