#ifndef GR_ACTION_HELPER_H
#define GR_ACTION_HELPER_H

namespace gr_safety_policies
{
    template <class T> 
    class ActionHelper{
        ActionHelper(bool negate){
            action_ = new T();

            if (negate){
                action_.stop();
            }
            else{
                action_.execute();
            }
        }

        protected:
            boost::shared_ptr<T> action_;

    };
}

#endif