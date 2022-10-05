import rospy
from std_msgs.msg import Time
import os

class Manager:
    def __init__(self, id="test"):
        try:
            os.mkdir(id)
        except:
            print("folder existed")
        self.test_id = id
        rospy.Subscriber("/start", Time,self.start_cb, queue_size=1)
        rospy.Subscriber("/stop", Time,self.stop_cb, queue_size=1)
        rospy.spin()

    def stop_cb(self,msg):
        with open('{}/stop.txt'.format(self.test_id),'a') as f:
            f.write(str(rospy.Time.now()) + "\n")

    def start_cb(self,msg):
        with open('{}/start.txt'.format(self.test_id),'a') as f:
            f.write(str(rospy.Time.now()) + "\n")
