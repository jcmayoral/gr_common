#!/usr/bin python
import rospy
import actionlib
from gr_action_msgs.msg import SimMotionPlannerAction, SimMotionPlannerGoal

class ActionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/SimMotionPlanner/my_person', SimMotionPlannerAction)
        self.client.wait_for_server()
        print ("Constructor")

    def send_goal(self):
        goal = SimMotionPlannerGoal()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        print ("end goal")


if __name__ == '__main__':
    rospy.init_node("experiment_sm")
    a = ActionClient()
    a.send_goal()
