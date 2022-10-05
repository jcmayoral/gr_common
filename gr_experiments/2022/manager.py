import rospy
from std_msgs.msg import Time
from safety_msgs.msg import HumanSafety
from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from configuration import ConfigurationManager
from std_srvs.srv import SetBool, SetBoolRequest
import os

class Manager:
    def __init__(self, id="test_results"):
        self.env_configuration_manager = ConfigurationManager(config_name="config/human_config.yaml")
        self.run_client = rospy.ServiceProxy('/execute_remotely', SetBool)
        self.env_dyn_client = Client("interface_for_person", timeout=30,config_callback=None)
        self.run_number = 1
        self.env_params = dict()
        self.run_finished = False
        rospy.Timer(rospy.Duration(15), self.timer_cb)


        try:
            os.mkdir(id)
        except:
            print("folder existed")
        self.test_id = id
        rospy.Subscriber("/start", Time,self.start_cb, queue_size=1)
        rospy.Subscriber("/stop", Time,self.stop_cb, queue_size=1)
        rospy.Subscriber("/my_person/human_collision", HumanSafety,self.collision_cb, queue_size=1)
    
    def timer_cb(self,event):
        print ('Timer called at ' + str(event.current_real))
        self.update_env_configuration()
    
    def run(self):
        self.run_finished = False
        run_msg = SetBoolRequest()

        for i in range(4):
            #START
            run_msg.data = True
            resp1 = self.run_client(run_msg)
            while not self.run_finished:
                rospy.sleep(0.1)

            run_msg.data = False
            resp1 = self.run_client(run_msg)

    def stop_cb(self,msg):
        with open('{}/stop.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(rospy.Time.now())))
        self.run_number +=1
        self.run_finished = True

    def start_cb(self,msg):
        with open('{}/start.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(rospy.Time.now())))

    def collision_cb(self,msg):
        #run_id time
        msg_str="{} {} ". format(str(self.run_number) ,str(rospy.Time.now()))
        #odom pose
        msg_str+="{} {} ".format(msg.odom_pose.point.x,msg.odom_pose.point.y)
        #base_link pose
        msg_str+="{} {} ".format(msg.base_link_pose.point.x,msg.base_link_pose.point.y)
        #distance 
        msg_str+="{}\n".format(msg.distance)
        

        with open('{}/collision.txt'.format(self.test_id),'a') as f:
            f.write(msg_str)


    def update_env_configuration(self):
        self.env_configuration_manager.get_new_param_values(self.env_params)
        rospy.loginfo("Updating Environment Configuration")
        try:
            self.env_dyn_client.update_configuration(self.env_params)
        except DynamicReconfigureCallbackException:
            rospy.logerr("Something goes wrong")
