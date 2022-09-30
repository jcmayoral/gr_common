#include <gr_safety_policies/utils/types_definitions.hpp>
#include <detection_msgs/BoundingBox.h>

namespace gr_safety_policies{
    float calculateIOU(const BoundingBoxInfo& lhs, const detection_msgs::BoundingBox& rhs){
        int xA = max(lhs.bb.x1, rhs->xmin)
        int yA = max(lhs.bb.y1, rhs->ymin)
        int xB = min(lhs.bb.x2, rhs->xmax)
        int yB = min(lhs.bb.y2, rhs->ymax)
        //compute the area of intersection rectangle
        auto interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1);
        //compute the area of both the prediction and ground-truth
        //rectangles
        auto boxAArea = (lhs.bb.x2 - lhs.bb.x1 + 1) * (lhs.bb.y2 - lhs.bb.y1 + 1);
        auto boxBArea = (rhs->xmax - rhs->xmin + 1) * (rhs->ymax - rhs->ymin + 1);
        //compute the intersection over union by taking the intersection
        //area and dividing it by the sum of prediction + ground-truth
        // areas - the interesection area
        auto iou = interArea / float(boxAArea + boxBArea - interArea);
        // return the intersection over union value
        return iou;
    }

    bool operator ==(const BoundingBoxInfo& lhs, const detection_msgs::BoundingBox& rhs){
        //Centroid distance ? 
        /*
        float rx = rhs->xmin + (rhs->xmax - rhs->ymin)/2;
        float ry = rhs->ymin + (rhs->ymax - rhs->ymin)/2;
        float centroids_distance = sqrt(pow(lhs.centroid_x - rx,2) + pow(lhs.centroid_y - ry,2));
        */
        float iou =  calculateIOU(lhs, rhs);

        if (iou < 0.75){
            return true;
        }

        return false;


        //IOU
    };

}
