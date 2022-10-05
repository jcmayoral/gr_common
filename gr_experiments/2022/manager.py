import rospy
from std_msgs.msg import Time
from safety_msgs.msg import HumanSafety
import os

class Manager:
    def __init__(self, id="test_results"):
        self.run_number = 1
        try:
            os.mkdir(id)
        except:
            print("folder existed")
        self.test_id = id
        rospy.Subscriber("/start", Time,self.start_cb, queue_size=1)
        rospy.Subscriber("/stop", Time,self.stop_cb, queue_size=1)
        rospy.Subscriber("/my_person/human_collision", HumanSafety,self.collision_cb, queue_size=1)
        rospy.spin()

    def stop_cb(self,msg):
        with open('{}/stop.txt'.format(self.test_id),'a') as f:
            f.write("{} {}\n". format(str(self.run_number) ,str(rospy.Time.now())))
        self.run_number +=1

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
