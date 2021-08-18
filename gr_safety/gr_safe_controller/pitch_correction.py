import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

class PitchCorrection:
    def __init__(self, pub_visualizer = False):
        rospy.loginfo("IIMU")
        self.imu_gain = 0.0
        self.imu_msg = Imu()
        rospy.Subscriber("/imu/filtered",Imu, self.imu_cb)
        if pub_visualizer:
            self.pub = rospy.Publisher("terrain_euler", PoseStamped)

    def imu_cb(self, msg):
        self.imu_msg = msg

    def publish_fb(self):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.pose.orientation = self.imu_msg.orientation
        self.pub.publish(msg)


if __name__ =="__main__":
    rospy.init_node("imu_experiments")
    pitch_correction = PitchCorrection(pub_visualizer=True)
    while not rospy.is_shutdown():
        pitch_correction.publish_fb()
        rospy.sleep(0.1)
