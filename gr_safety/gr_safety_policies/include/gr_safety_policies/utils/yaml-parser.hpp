#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <gr_safety_policies/utils/transition_structure.hpp>

using namespace gr_safety_policies;

TransitionArray parseFile(std::string config_file){
    //YAML-CPP
    YAML::Node node;
    std::string path = ros::package::getPath("gr_safety_policies");
    YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_file).c_str());
    node = config_yaml["states"];
    int number_states = node.size();

    //Transitions
    TransitionArray array;
    //TransitionInfo info;
    //info.name = "action";
    //info.negate = true;
    //array["Lethal"]["Lethal"] = info; 

    std::cout << "Parsing file " << path+"/"+config_file <<  std::endl;
    for (YAML::const_iterator a= node.begin(); a != node.end(); ++a){
        //std::string name = a->first.as<std::string>();
        //YAML::Node config = a->second;
        std::string name = a->as<std::string>();
        std::cout << name << std::endl;
        //strategy_selector_[name] = config["strategy"].as<std::string>();
        //config["strategy"]>> strategy_selector_[name];
    }

    node = config_yaml["transitions"];
    for (YAML::const_iterator a= node.begin(); a != node.end(); ++a){
        YAML::Node transition = a->second;
        TTransition info;
        std::string state1 = a->first.as<std::string>();
        //Each cases
        for (YAML::const_iterator b= transition.begin(); b != transition.end(); ++b){
            auto state2 = b->first.as<std::string>();
            auto action = b->second.as<std::string>();

            info[state2].negate = false;
            std::size_t found = action.find("!");
            if (found!=std::string::npos){
                info[state2].negate = true;
                action.erase(found);
                //std::cout << "first 'needle' found at: " << found << " action " << action << '\n';
            }
            info[state2].action = action;
        }
        array[state1] = info;
    }
    return array;
}