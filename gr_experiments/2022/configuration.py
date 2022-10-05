import yaml
import copy
import rospy
import random

class ConfigurationManager:
    def __init__(self, config_name = 'config/human_config.yaml', static_dict=None):
        file_stream = open(config_name, 'r')
        params = yaml.load(file_stream)

        self.config_params = dict()
        self.static_dict = dict()

        if static_dict is not None:
            self.static_dict.update(static_dict)

        for i in params:
            self.config_params[i] = Param(**params[i])

        self.default_config = copy.deepcopy(self.config_params)
        rospy.loginfo("Configuration Manager is ready")

    def get_default_config(self):
        new_param_dict = dict()

        for i in self.config_params:
            new_param_dict[i] = self.default_config[i].get_current_value()

        return new_param_dict

    def restart_params(self):
        self.config_params = copy.deepcopy(self.default_config)

    def get_new_param_value(self, key):
        return self.config_params[key].get_random_value()

    def get_new_param_values(self, new_param_dict):
        for i in self.config_params:
            new_param_dict[i] = self.get_new_param_value(i)
        new_param_dict.update(self.static_dict)




class Param:
    def __init__(self, min_value, max_value, increment=0.1, default_value = None):
        self.min_value = min_value
        self.max_value = max_value
        self.increment = increment

        #TODO Get Default values from param server
        if default_value is None:
            self.last_value = (max_value - min_value)/2 #Initializing some point in the middle
        else:
            self.last_value = default_value

    def get_current_value(self):
        return self.last_value

    def get_random_value(self):
        random_flag = random.randint(0,2)

        if random_flag == 0: #NO MODIFICATIONS
            return self.last_value
        elif random_flag == 1: #INCREMENT
            self.last_value = min(self.increment + self.last_value, self.max_value)
            return self.last_value
        else: #DECREASE VALUE
            self.last_value = max(self.last_value - self.increment, self.min_value)
            return self.last_value
