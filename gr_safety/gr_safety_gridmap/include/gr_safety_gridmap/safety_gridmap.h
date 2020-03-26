#include <ros/ros.h>

namespace gr_safety_gridmap{
    class PoseTracker{
        private:
            ros::Subscriber sub_;
        public:
            ros::Publisher pub_;


}