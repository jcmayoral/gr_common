#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class SafeController:
    def __init__(self):
        rospy.init_node("safe_controller")
        self.penalization_factor = 0.0
        self.twist = Twist()
        self.cmd_pub = rospy.Publisher("/safe_nav_vel", Twist,queue_size=1)
        #change to message filter?
        rospy.Subscriber("/safety_score",Float32, self.safety_cb)
        rospy.Subscriber("/nav_vel",Twist, self.vel_cb)

        #rospy.Timer(rospy.Duration(0.05), self.timer_event)
        rospy.spin()

    def timer_event(self,event):
        self.twist.linear.x = self.twist.linear.x * (1-self.penalization_factor)
        self.cmd_pub.publish(self.twist)

    def vel_cb(self,msg):
        self.twist = msg
        self.twist.linear.x = self.twist.linear.x * (1-self.penalization_factor)
        self.cmd_pub.publish(self.twist)


    def safety_cb(self,msg):
        self.penalization_factor = msg.data
