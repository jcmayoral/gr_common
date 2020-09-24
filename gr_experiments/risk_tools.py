from metrics import *
from std_msgs.msg import Empty, Float32
from safety_msgs.msg import FoundObjectsArray
from scipy.spatial import distance
import rospy
from numpy import cov
import message_filters

class RiskExtractor:
    def __init__(self, file_name):
        rospy.init_node("gr_experiment_tool")
        self.file_name = file_name
        self.obstacles = []#dict()
        self.safety_scores = []
        self.run = True
        #self.subscriber = rospy.Subscriber("/pointcloud_lidar_processing/found_object", FoundObjectsArray, self.cb)
        self.subscriber2 = rospy.Subscriber("/test_finished", Empty, self.test_cb)
        #self.subscriber3 = rospy.Subscriber("/safety_score", Float32, self.cb2)

        sub1 = message_filters.Subscriber("/pointcloud_lidar_processing/found_object", FoundObjectsArray)
        sub2 = message_filters.Subscriber("/safety_score", Float32)
        self.ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], queue_size=10, slop=0.5, allow_headerless=True)
        self.ts.registerCallback(self.timed_cb)

        while not rospy.is_shutdown() and self.run:
            pass

        #print "OBSTACLES"
        self.execute(self.obstacles, "OBSTACLES")
        #print "SAFETY SCORES ", self.safety_scores
        self.execute(self.safety_scores, "SCORE")


    def timed_cb(self, persons, score):
        print "timed cb", len(self.obstacles), len(self.safety_scores)
        #print "error ", len(persons.objects) <1
        if len(persons.objects) <1:
            return
        self.cb(persons)
        self.cb2(score)

    def test_cb(self, flag):
        print "out"
        self.run = False
        #self.subscriber.unregister()
        #self.subscriber3.unregister()

    def cb2(self, score):
        self.safety_scores.append(score.data)

    def cb(self, persons):
        #Note distance on velodyne fixed
        #TODO MODIFY to generalisze
        #for p in persons.objects:
        #    self.obstacles.append(distance.euclidean((p.pose.position.x, p.pose.position.y, p.pose.position.z), (0,0,0)))
        p = persons.objects[0]
        self.obstacles.append(distance.euclidean((p.pose.position.x, p.pose.position.y, p.pose.position.z), (0,0,0)))

    def execute(self, data, id):
        f = open(self.file_name, "w")
        f.write(id)
        f.write("Number of detections " + str(len(data))+"\n")
        if len(data)> 0:
            f.write("SM1 Metric: " + str(SM1(data))+"\n")
            f.write("SM2 Metric: " + str(SM2(data))+"\n")
            f.write("SM3 Metric: " + str(SM3(data))+"\n")
            f.write("COVARIANCE " + str(cov(self.obstacles, self.safety_scores))+"\n")
        f.close()
