#!/usr/bin/python
import yaml
import rospy
import sys
from dynamic_reconfigure.client import Client
from std_msgs.srv import Empty, EmptyResponse, EmptyRequest

class DynReconfigurator:
    def __init__(self):
        rospy.init_node("dynamic_reconfigure_experimnetal")
        self.configuration = dict()
        self.client = Client("gr_safe_gridmap_node", timeout=30, config_callback=self.callback)
        self.service = rospy.Service("set_configuration", Empty, self.set_configuration)

    def set_configuration(self, req):
        #TODO create a list of  config
        self.load('dynrec_sample.yaml')


    def callback(self, config):
        print "updated", config

    def save(self):
        with file("test.yaml", 'w') as f:
            yaml.dump(self.configuration,f)

    #from https://github.com/ros/dynamic_reconfigure/blob/noetic-devel/scripts/dynparam
    #otherwise run a pipeline
    def load(self, filename):
        f = open(filename, 'r')
        try:
            self.configuration = {}
            for doc in yaml.load_all(f.read()):
                self.configuration.update(doc)
        finally:
            f.close()
        print "LOADING ", self.configuration

        #connect()
        #set_params(node, params, timeout=options.timeout)

    def update(self):
        """a = {'selection': 'hola2', 'timestep': 0.5, 'nprimitives': 3, 'tracking': 5, 'mode': 1,
            'groups': {'selection': 'hola', 'parent': 0, 'timestep': 0.5, 'nprimitives': 3,
            'groups': {},
            'id': 0, 'tracking': 5, 'name': 'Default', 'parameters': {},
            'sensors_ids': '/0', 'state': True, 'mode': 1, 'enabler': False, 'type': ''},
            'enabler': False, 'sensors_ids': '/10'}
        print type(a), type(self.configuration)
        """

        self.client.update_configuration(self.configuration)




if __name__ == '__main__':
    print rospy.myargv()
    print "DEPRECATED use navigation_experiment"
    sys.exit()
    dyn = DynReconfigurator()
    #dyn.save()
    #dyn.load('dynrec_sample.yaml')
    #dyn.update()
