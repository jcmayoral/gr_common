#include<string>

namespace gr_safety_policies{
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

    struct TransitionsManager
    {
        public:
            std::string name;
            std::map<std::string,Transition*> transitions;
        /* data */
    };

}
