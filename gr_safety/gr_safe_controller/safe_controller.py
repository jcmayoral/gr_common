#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class SafeController:
    def __init__(self):
        rospy.init_node("safe_controller")
        self.lin_speed = 0.8
        self.penalization_factor = 0.0
        self.cmd_pub = rospy.Publisher("/nav_vel", Twist)
        rospy.Subscriber("/safety_score",Float32, self.safety_cb)
        rospy.Timer(rospy.Duration(0.1), self.timer_event)
        rospy.spin()

    def timer_event(self,event):
        twist = Twist()
        twist.linear.x = self.lin_speed * (1-self.penalization_factor)
        rospy.loginfo(twist)
        self.cmd_pub.publish(twist)

    def safety_cb(self,msg):
        self.penalization_factor = msg.data
