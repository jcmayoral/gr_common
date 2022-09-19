#include<string>

namespace gr_safety_policies{
    //typedef TransitionInfo = std::tuple<std::string, bool>
    struct TransitionInfo {
        std::string name;
        bool negate; 
    };

    typedef std::map<std::string, TransitionInfo> TTransition;
    typedef std::map<std::string, TTransition> TransitionArray;

    class Transition{
        std::string name_;
        bool negate_;
        
        public:
            Transition():
                name_{"default"}, negate_{false}
            {

            }

            std::string getName(){
                return name_;
            }

            bool isNegated(){
                return negate_;
            }

            void setName(std::string name){
                name_ = name;
            }


    };


    /*
    constexpr T& operator[](std::string name){
        return a[z * Y * X + y * X + x];
    }
    */
        

    struct TransitionsManager
    {
        public:
            std::string name;
            std::map<std::string,Transition*> transitions;
    };

}
