#include<string>

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

    struct BoundingBox{
        int x1;
        int y1;
        int x2;
        int y2;
    }

    struct BoundingBoxInfo{
        int centroid_x;
        int centroid_y;
        BoundingBox bb;
    }
}
