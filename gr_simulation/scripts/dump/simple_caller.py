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
