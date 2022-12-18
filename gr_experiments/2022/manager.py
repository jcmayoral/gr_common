import rospy
from std_msgs.msg import Time, Empty
from safety_msgs.msg import HumanSafety, StateTransition
from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from configuration import ConfigurationManager
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerResponse
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry

import os
import tqdm
import time
import copy

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

        self.hri_requested = False

        rospy.Subscriber("/hri_requested", Empty, self.hri_cb, queue_size=1)

        rospy.Subscriber("/start", Time,self.start_cb, queue_size=1)
        rospy.Subscriber("/stop", Time,self.estop_cb, queue_size=1)

        indexes = ["", "_0"]
        for i in indexes:
            self.env_dyn_client.append(Client("interface_for_person{}".format(i), timeout=30,config_callback=None))
            self.env_configuration_manager.append(ConfigurationManager(config_name="config/human_config{}.yaml".format(i)))
            self.env_params.append(dict())
            rospy.Subscriber("/my_person{}/human_collision".format(i), HumanSafety,callback_args=i,callback=self.collision_cb, queue_size=1)
            rospy.Subscriber("/my_person{}/human_trigger".format(i), Empty,callback_args=i,callback=self.start_human, queue_size=1)

        rospy.Subscriber("/bumper_vals", ContactsState, queue_size=1, callback=self.bumper_vals)
        rospy.Subscriber("/odometry/base_raw", Odometry, queue_size=1, callback=self.odom_cb)
        rospy.Subscriber('/feedback', StateTransition, self.st_cb)

        rospy.Timer(rospy.Duration(15), self.timer_cb)

    def st_cb(self, msg):
        with open('{}/state_transitions.txt'.format(self.test_id),'a') as f:
            f.write("{} {} {}\n". format(str(time.time()), msg.previous, msg.current))

    def hri_cb(self, msg):
        print ("hri cb")
        with open('{}/hri_requests.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(time.time())))

    def timer_cb(self,event):
        #print ('Timer called at ' + str(event.current_real))
        self.update_env_configuration(0)
        self.update_env_configuration(1)

    def start_human(self,event,id):
        #print ("restart person")
        idx = 0
        if id == "_0":
            idx = 1
        self.update_env_configuration(idx)

    def bumper_vals(self,msg):
        if len(msg.states) ==0:
            return
        self.update_env_configuration(0)
        self.update_env_configuration(1)

    def run(self, repetitions):
        self.run_finished = False
        run_msg = SetBoolRequest()
        i =0
        resp1 = self.run_client(run_msg)
        rospy.sleep(1.0)

        #for i in tqdm.tqdm(range(repetitions)):
        while i < repetitions:
            print ("{} of {}".format(i, repetitions))
            #START
            self.run_number = i
            #print("RUN {}". format(i))
            run_msg.data = True
            self.run_client.call(run_msg)
            rospy.sleep(0.4)
            self.run_finished = False

            while not self.run_finished:
                rospy.sleep(0.1)


            print ("end or abort")
            #self.run_finished = False

            print("end")
            i=i+1
            #rospy.logwarn("Restarting")
            #run_msg.data = False
            #resp1 = self.run_client(run_msg)
            self.stop_cb()
        self.has_finished = True

    def stop_cb(self):
        with open('{}/stop.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(time.time())))

    def estop_cb(self,msg):
        print("estop")
        with open('{}/estop.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(time.time())))
        self.run_finished = True

    def start_cb(self,msg):
        with open('{}/start.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(time.time())))

    def odom_cb(self,msg):
        #run_id time
        msg_str="{} {} ". format(str(self.run_number) ,str(time.time()))
        #odom pose
        msg_str+="{} {} ".format(msg.pose.pose.position.x,msg.pose.pose.position.y)
        #odom quat
        msg_str+="{} {} {} {} ".format(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w )
        #velocity
        msg_str+="{} {} {}\n".format(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)


        with open('{}/odom.txt'.format(self.test_id, id),'a') as f:
            f.write(msg_str)

    def collision_cb(self,msg, id):
        #run_id time
        msg_str="{} {} ". format(str(self.run_number) ,str(time.time()))
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
