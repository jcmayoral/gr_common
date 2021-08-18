import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import tf
import matplotlib.pyplot as plt

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

    def get_euler_angles(self):
        quaternion = (
            self.imu_msg.orientation.x,
            self.imu_msg.orientation.y,
            self.imu_msg.orientation.z,
            self.imu_msg.orientation.w)
        return tf.transformations.euler_from_quaternion(quaternion)



if __name__ =="__main__":
    plt.ion()
    import numpy as np
    i = 0
    maxsize = 10
    dt = np.arange(maxsize)
    data = np.zeros((maxsize,3))

    rospy.init_node("imu_experiments")
    pitch_correction = PitchCorrection(pub_visualizer=True)
    while not rospy.is_shutdown():
        ax = plt.axes()

        pitch_correction.publish_fb()
        if i >= maxsize:
            i=0
        data[i]  = pitch_correction.get_euler_angles()
        i = i+1
        ax.plot(dt, data[:,0])
        ax.plot(dt, data[:,1])
        ax.plot(dt, data[:,2])

        plt.draw()
        plt.pause(0.1)
        plt.cla()
        rospy.sleep(0.1)
        #plt.clf()
