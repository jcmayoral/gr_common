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
        self.current_state = -1
        self.levels=["SAFE", "WARNING", "DANGER"]
        self.file_root = "/home/jose/.safe_sounds"
        self.ring_start()
        rospy.Subscriber("/safety_score", Float32, self.safe_cb)
        rospy.spin()

    def ring_start(self):
        path = os.path.join(self.file_root, "START")
        file_ = random.choice(os.listdir(path))
        playsound(os.path.join(path,file_))
        self.current_state = 0
        #time.sleep(0.1)


    def safe_cb(self, msg):
        index = int(msg.data*(len(self.levels)-1))
        print index, msg.data

        if index == self.current_state:
            rospy.loginfo("avoid state")
            return
        path = os.path.join(self.file_root, self.levels[self.current_state], self.levels[index])
        print path
        file_ = random.choice(os.listdir(path))
        playsound(os.path.join(path,file_))
        self.current_state = index
        #time.sleep(0.)



if __name__ == "__main__":
    ROSPlaySound()
