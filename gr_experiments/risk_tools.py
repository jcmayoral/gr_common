from metrics import *
from std_msgs.msg import Empty
from safety_msgs.msg import FoundObjectsArray
from scipy.spatial import distance
import rospy

class RiskExtractor:
    def __init__(self):
        rospy.init_node("gr_experiment_tool")
        self.obstacles = []#dict()
        self.run = True
        self.subscriber = rospy.Subscriber("/pointcloud_lidar_processing/found_object", FoundObjectsArray, self.cb)
        self.subscriber2 = rospy.Subscriber("/test_finished", Empty, self.test_cb)

        while not rospy.is_shutdown() and self.run:
            pass

        self.execute()

    def test_cb(self, flag):
        print "out"
        self.run = False
        self.subscriber.unregister()

    def cb(self, persons):
        #Note distance on velodyne fixed
        #TODO MODIFY to generalisze
        for p in persons.objects:
            self.obstacles.append(distance.euclidean((p.pose.position.x, p.pose.position.y, p.pose.position.z), (0,0,0)))

    def execute(self):
        print "Number of detections " , len(self.obstacles)
        print "SM1 Metric: " , SM1(self.obstacles)
        print "SM2 Metric: " , SM2(self.obstacles)
        print "SM3 Metric: " , SM3(self.obstacles)
