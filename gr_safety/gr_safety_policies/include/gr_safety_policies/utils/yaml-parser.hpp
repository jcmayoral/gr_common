#include <yaml-cpp/yaml.h>
#include <ros/ros.h>


using namespace gr_safety_policies;

void parseFile(std::string config_file){
    YAML::Node node;
    std::string path = ros::package::getPath("gr_safety_policies");
    YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_file).c_str());

    node = config_yaml["states"];
    int number_states = node.size();

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
        std::cout << "state " << a->first   << std::endl;
        //Each cases
        for (YAML::const_iterator b= transition.begin(); b != transition.end(); ++b){
            std::cout << "transiiton "<< b->first.as<std::string>() << b->second.as<std::string>() << std::endl;
        }
    }
 

}