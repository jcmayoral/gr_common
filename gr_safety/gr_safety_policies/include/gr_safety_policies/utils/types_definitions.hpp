#include<string>
#include <detection_msgs/BoundingBox.h>

namespace gr_safety_policies{
    //typedef TransitionInfo = std::tuple<std::string, bool>
    struct TransitionInfo {
        std::string action="None";
        bool negate = false; 
    };

    typedef std::map<std::string, TransitionInfo> TTransition;
    typedef std::map<std::string, TTransition> TransitionArray;
    typedef std::map<std::string, int> RiskLevels;

    struct RiskManager {
        TransitionArray transition;
        RiskLevels levels;
    };

    struct MyBoundingBox{
        int x1 = 0;
        int y1 = 0;
        int x2 = 0;
        int y2 = 0;
    };

    class BoundingBoxInfo{
        public:
            int centroid_x;
            int centroid_y;
            MyBoundingBox bb;
            void updateObject(const detection_msgs::BoundingBox* detec){
                bb.x1 = detec->xmin;
                bb.x2 = detec->xmax;
                bb.y1 = detec->ymin;
                bb.y2 = detec->ymax;
            }
    };
}
