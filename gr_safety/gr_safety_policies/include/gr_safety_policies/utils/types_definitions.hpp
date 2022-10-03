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
        int x1 = std::numeric_limits<int>::max();;
        int y1 = std::numeric_limits<int>::max();;
        int x2 = std::numeric_limits<int>::min();
        int y2 = std::numeric_limits<int>::min();
    };

    class BoundingBoxInfo{
        public:
            BoundingBoxInfo():bb{}{
            }

            friend std::ostream &operator<<(std::ostream &os, const BoundingBoxInfo* m) {
                //os << "Bounding BOX "<< m.bb.x1 << " " << m.bb.y1 << " " << m.bb.x2 << " " << m.bb.y2;
                os << "xmin " << m->bb.x1 << " xmax "<< m->bb.x2 << " ymin " << m->bb.y1 << " ymax " << m->bb.y2;
                return os;
            }

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
