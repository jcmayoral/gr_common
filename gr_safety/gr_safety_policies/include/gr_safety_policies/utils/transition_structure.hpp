#include<string>

namespace gr_safety_policies{
    //typedef TransitionInfo = std::tuple<std::string, bool>
    struct TransitionInfo {
        std::string action;
        bool negate = false; 
    };

    typedef std::map<std::string, TransitionInfo> TTransition;
    typedef std::map<std::string, TTransition> TransitionArray;
}
