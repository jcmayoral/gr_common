#ifndef GR_ACTION_HELPER_H
#define GR_ACTION_HELPER_H

namespace gr_safety_policies
{
    template <class T> 
    class ActionHelper{
        ActionHelper(){
            action_ = new T();
        }

        protected:
            boost::shared_ptr<T> action_;

    };
}

#endif