#!/usr/bin/python
import rospy
from playsound import playsound
from std_msgs.msg import Float32
import os
import random
import time

class ROSPlaySound:
    def __init__(self):
        rospy.init_node("ros_play_sound")
        self.levels=["SAFE", "WARNING", "DANGER"]
        self.file_root = "/home/jose/.safe_sounds"
        rospy.Subscriber("/safety_score", Float32, self.safe_cb)
        rospy.spin()

    def safe_cb(self, msg):
        index = int(msg.data/len(self.levels))
        path = os.path.join(self.file_root, self.levels[index])
        file_ = random.choice(os.listdir(path))
        playsound(os.path.join(path,file_))
        time.sleep(1)



if __name__ == "__main__":
    ROSPlaySound()
