import rospy
from manager import Manager


if __name__ == "__main__":
    rospy.init_node("experiments")
    manager = Manager()

    while not rospy.is_shutdown() and not manager.has_finished:
        manager.run()

    rospy.loginfo("experiments finished")
