#ifndef DYNAMIC_RECONFIGURE_SAFE_ACTION_H
#define DYNAMIC_RECONFIGURE_SAFE_ACTION_H

#include <safety_core/safe_action.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

namespace gr_safety_policies{
    class ReconfigureControllerSafeAction : public safety_core::SafeAction{
        public:
            ReconfigureControllerSafeAction();
            ~ReconfigureControllerSafeAction();
            virtual void execute();
            virtual void stop();
    };
};

#endif
