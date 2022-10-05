import rospy
from std_msgs.msg import Time, Empty
from safety_msgs.msg import HumanSafety
from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from configuration import ConfigurationManager
from std_srvs.srv import SetBool, SetBoolRequest
import os
import tqdm

class Manager:
    def __init__(self, id="test_results"):
        self.run_client = rospy.ServiceProxy('/execute_remotely', SetBool)
        self.env_dyn_client = list()
        self.run_number = 1

        self.env_params = list()
        self.env_configuration_manager = list()
       
        self.run_finished = False
        self.has_finished = False        

        try:
            os.mkdir(id)
        except:
            print("folder existed")
        self.test_id = id
        rospy.Subscriber("/start", Time,self.start_cb, queue_size=1)
        rospy.Subscriber("/stop", Time,self.estop_cb, queue_size=1)


        indexes = ["", "_0"]
        for i in indexes:
            self.env_dyn_client.append(Client("interface_for_person{}".format(i), timeout=30,config_callback=None))
            self.env_configuration_manager.append(ConfigurationManager(config_name="config/human_config{}.yaml".format(i)))
            self.env_params.append(dict())
            rospy.Subscriber("/my_person{}/human_collision".format(i), HumanSafety,callback_args=i,callback=self.collision_cb, queue_size=1)
            rospy.Subscriber("/my_person{}/human_trigger".format(i), Empty,callback_args=i,callback=self.start_human, queue_size=1)
        
        rospy.Timer(rospy.Duration(15), self.timer_cb)

    def timer_cb(self,event):
        #print ('Timer called at ' + str(event.current_real))
        self.update_env_configuration(0)
        self.update_env_configuration(1)

    def start_human(self,event,id):
        #print ("restart person")
        self.update_env_configuration(0)
        self.update_env_configuration(1)


    def run(self, repetitions):
        self.run_finished = False
        run_msg = SetBoolRequest()

        for i in tqdm.tqdm(range(repetitions)):
            #START
            self.run_number = i
            #print("RUN {}". format(i))
            run_msg.data = True
            resp1 = self.run_client(run_msg)
            while not self.run_finished:
                rospy.sleep(0.1)
            self.run_finished = False
            #rospy.logwarn("Restarting")
            #run_msg.data = False
            #resp1 = self.run_client(run_msg)
            self.stop_cb()
        self.has_finished = True


    def stop_cb(self):
        with open('{}/stop.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(rospy.Time.now())))

    def estop_cb(self,msg):
        with open('{}/estop.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(rospy.Time.now())))
        self.run_finished = True

    def start_cb(self,msg):
        with open('{}/start.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(rospy.Time.now())))

    def collision_cb(self,msg, id):
        #run_id time
        msg_str="{} {} ". format(str(self.run_number) ,str(rospy.Time.now()))
        #odom pose
        msg_str+="{} {} ".format(msg.odom_pose.point.x,msg.odom_pose.point.y)
        #base_link pose
        msg_str+="{} {} ".format(msg.base_link_pose.point.x,msg.base_link_pose.point.y)
        #distance 
        msg_str+="{}\n".format(msg.distance)
        

        with open('{}/collision{}.txt'.format(self.test_id, id),'a') as f:
            f.write(msg_str)


    def update_env_configuration(self, id):
        self.env_configuration_manager[id].get_new_param_values(self.env_params[id])
        #rospy.loginfo("Updating Environment Configuration")
        try:
            self.env_dyn_client[id].update_configuration(self.env_params[id])
        except DynamicReconfigureCallbackException:
            rospy.logerr("Something goes wrong")
