from metrics import *
from std_msgs.msg import Empty, Float32
from safety_msgs.msg import FoundObjectsArray
from scipy.spatial import distance
import rospy
from numpy import cov
import message_filters
from gazebo_msgs.msg import ContactsState
#import actionlib
from gr_action_msgs.srv import GetMetrics, GetMetricsResponse

class RiskExtractor:
    def __init__(self, file_name, service_required = False):
        rospy.init_node("gr_experiment_tool")
        self.file_name = file_name
        self.collision_detected = False
        self.reset()
        self.run = True
        #self.subscriber = rospy.Subscriber("/pointcloud_lidar_processing/found_object", FoundObjectsArray, self.cb)
        self.subscriber2 = rospy.Subscriber("/test_finished", Empty, self.test_cb)
        #self.subscriber3 = rospy.Subscriber("/safety_score", Float32, self.cb2)
        self.subscriber4 = rospy.Subscriber("/bumper_vals", ContactsState, self.collision_cb)

        sub1 = message_filters.Subscriber("/pointcloud_lidar_processing/found_object", FoundObjectsArray)
        sub2 = message_filters.Subscriber("/safety_score", Float32)
        self.ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], queue_size=10, slop=0.5, allow_headerless=True)
        self.ts.registerCallback(self.timed_cb)


        if not service_required:
            while not rospy.is_shutdown() and self.run:
                pass
            #print "OBSTACLES"
            self.execute(self.obstacles, "OBSTACLES")
            #print "SAFETY SCORES ", self.safety_scores
            self.execute(self.safety_scores, "SCORE")
        else:
            #TODO SERVICE?
            #self._as = actionlib.SimpleActionServer("SingleRowExecutionAction", SingleRowExecutionAction, execute_cb=self.execute_cb, auto_start = False)
            #self._as.start()
            service = rospy.Service("get_metrics", GetMetrics, self.get_metrics)
            rospy.spin()

    def collision_cb(self,msg):

        for state in msg.states:
            if "ground" not in state.collision1_name and "ground" not in state.collision2_name:
                print "COL detected"
                #rospy.logerr(state)
                rospy.logerr(len(msg.states))
                self.collision_detected = True
            #else:
            #    print "OK"

    def get_metrics(self, req):
        print "SERVICE REQUIRED, " , req.file_name
        self.file_name = req.file_name + ".txt"
        r = GetMetricsResponse()
        m1 = self.generate_metric(self.obstacles)
        m2 = self.generate_metric(self.safety_scores)
        r.metrics.extend(m1)
        r.metrics.extend(m2)

        in_collision = [int(self.collision_detected)]
        r.metrics.extend(in_collision)
        print r.metrics

        #savefiles
        self.execute(self.obstacles, "OBSTACLES")
        self.execute(self.safety_scores, "SCORE")
        self.reset()

        return r

    #def execute_cb(self,goal):
    def reset(self):
        self.obstacles = []#dict()
        self.safety_scores = []
        self.collision_detected = False

    def timed_cb(self, persons, score):
        rospy.loginfo_throttle(10, "timed cb " + str(len(self.obstacles)) + str(len(self.safety_scores)))
        #print "error ", len(persons.objects) <1
        if len(persons.objects) == 0:
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
        f = open(self.file_name, "a")
        f.write(id+"\n")
        f.write("Number of detections " + str(len(data))+"\n")
        if len(data)> 0:
            f.write("SM1 Metric: " + str(SM1(data))+"\n")
            f.write("SM2 Metric: " + str(SM2(data))+"\n")
            f.write("SM3 Metric: " + str(SM3(data))+"\n")
            f.write("COVARIANCE " + str(cov(self.obstacles, self.safety_scores))+"\n")
        f.close()

    def generate_metric(self, data):
        print "DATA", data, len(data)
        if len(data)> 0:
            data = [float(len(data)),float(SM1(data)),float(SM2(data)),float(SM3(data))]
        else:
            data = [100000.0,100000.0,100000.0,100000.0]
        print data
        return data
