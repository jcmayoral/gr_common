#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <gr_safety_gridmap/layer_subscriber.hpp>
#include <yaml-cpp/yaml.h>
#include <safety_msgs/RiskIndexes.h>

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
            ros::Publisher request_stop_;
            ros::Publisher objects_risk_pub_;
            std::vector<LayerSubscriber> layer_subscribers;
            ros::Timer clear_timer_;
            float safety_threshold_;
            boost::mutex smtx;
    };

};
