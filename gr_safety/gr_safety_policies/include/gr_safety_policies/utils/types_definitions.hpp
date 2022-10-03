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
        int x2 = 100000;
        int y2 = 100000;
    };

    class BoundingBoxInfo{
        public:
            BoundingBoxInfo():bb{}{
            }

            friend std::ostream &operator<<(std::ostream &os, const BoundingBoxInfo* m) {
                //os << "Bounding BOX "<< m.bb.x1 << " " << m.bb.y1 << " " << m.bb.x2 << " " << m.bb.y2;
                os << "xmin " << m->bb.x1 << " xmax "<< m->bb.x2 << " ymin " << m->bb.y1 << " ymax " << m->bb.y2
                    << " cx " << m->centroid_x << " cy"  << m->centroid_y;
                return os;
            }

            int centroid_x = 10000;
            int centroid_y = 10000;
            bool is_started = false;

            MyBoundingBox bb;
            void updateObject(const detection_msgs::BoundingBox* detec){
                is_started = true;
                bb.x1 = detec->xmin;
                bb.x2 = detec->xmax;
                bb.y1 = detec->ymin;
                bb.y2 = detec->ymax;

                centroid_x = detec->xmin + (detec->xmax - detec->xmin)/2;
                centroid_y = detec->ymin + (detec->ymax - detec->ymin)/2;
            }
    };

    
}
