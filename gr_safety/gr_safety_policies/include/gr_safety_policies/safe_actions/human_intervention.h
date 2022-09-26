#ifndef SERVICE_CALLER_SAFE_ACTION_H
#define SERVICE_CALLER_SAFE_ACTION_H

#include <safety_core/safe_action.h>
#include <ros/ros.h>
#include<std_srvs/Trigger.h>
#include<std_srvs/TriggerResponse.h>


namespace gr_safety_policies{
    class HumanInterventionSafeAction : public safety_core::SafeAction{
        public:
            HumanInterventionSafeAction();
            ~HumanInterventionSafeAction();
            virtual void execute();
            virtual void stop();
    };
};

#endif
