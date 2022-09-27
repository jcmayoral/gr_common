#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from pitch_correction import PitchCorrection
from std_srvs.srv import SetBool, SetBoolResponse

class SafeController(PitchCorrection):
    def __init__(self):
        rospy.init_node("safe_controller")
        PitchCorrection.__init__(self)
        self.limit_speed = False
        self.s = rospy.Service('/limit_safe_speed', SetBool, self.safe_speed_cb)

        self.penalization_factor = 0.0
        self.twist = Twist()
        #self.cmd_pub = rospy.Publisher("/nav_vel", Twist,queue_size=1)
        self.cmd_pub = rospy.Publisher("/safe_nav_vel", Twist ,queue_size=1)
        #change to message filter?
        rospy.Subscriber("/safety_score",Float32, self.safety_cb)
        rospy.Subscriber("/movebase_vel",Twist, self.vel_cb)

        #rospy.Timer(rospy.Duration(0.05), self.timer_event)
        rospy.spin()
    
    def safe_speed_cb(self,req):
        print("limit_speed ", req.data)
        if req.data:
            self.limit_speed = True
        return SetBoolResponse(success=True)

    def timer_event(self,event):
        self.twist.linear.x = self.twist.linear.x * (1-self.penalization_factor)
        self.cmd_pub.publish(self.twist)

    def vel_cb(self,msg):
        #print (self.get_vel_factor(), self.penalization_factor)
        self.publish_fb()
        self.twist = msg
        self.twist.linear.x = self.twist.linear.x * self.get_vel_factor() * (1-self.penalization_factor)

        if self.limit_speed:
            self.limit_speed = min(self.safe_speed, self.limit_speed)
        self.cmd_pub.publish(self.twist)

    def safety_cb(self,msg):
        self.penalization_factor = msg.data



if __name__ == '__main__':
    SafeController()
