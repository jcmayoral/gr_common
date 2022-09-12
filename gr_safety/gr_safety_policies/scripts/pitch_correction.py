import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import tf
import matplotlib.pyplot as plt
import numpy as np

class PitchCorrection:
    def __init__(self):
        rospy.loginfo("IIMU")
        self.imu_gain = 0.0
        self.imu_msg = Imu()
        self.euler_angles = [0,0,0]
        self.pub = rospy.Publisher("terrain_euler", PoseStamped)
        rospy.Subscriber("/imu/filtered",Imu, self.imu_cb)


    def imu_cb(self, msg):
        self.imu_msg = msg

    def publish_fb(self):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.pose.orientation = self.imu_msg.orientation
        self.pub.publish(msg)

    def update_euler_angles(self):
        quaternion = (
            self.imu_msg.orientation.x,
            self.imu_msg.orientation.y,
            self.imu_msg.orientation.z,
            self.imu_msg.orientation.w)
        self.euler_angles =  tf.transformations.euler_from_quaternion(quaternion)
        return self.euler_angles

    def get_vel_factor(self):
        self.update_euler_angles()
        return np.cos(self.euler_angles[1])



if __name__ =="__main__":
    plt.ion()
    import numpy as np
    i = 0
    maxsize = 25
    dt = np.arange(maxsize)
    data = np.zeros((maxsize,4))

    rospy.init_node("imu_experiments")
    pitch_correction = PitchCorrection()

    fig, (ax1, ax2) = plt.subplots(2)
    fig.suptitle('Vertically stacked subplots')


    while not rospy.is_shutdown():
        #ax = plt.axes()
        ax2.set_ylim([-0.1,1.1])

        pitch_correction.publish_fb()
        if i >= maxsize:
            i=0
        data[i,:3]  = pitch_correction.update_euler_angles()
        data[i,3] = pitch_correction.get_vel_factor()
        i = i+1
        ax1.plot(dt, data[:,0])
        ax1.plot(dt, data[:,1])
        ax1.plot(dt, data[:,2])
        ax2.plot(dt, data[:,3])

        plt.draw()
        plt.pause(0.1)
        ax1.cla()
        ax2.cla()
        rospy.sleep(0.1)
        #plt.clf()
