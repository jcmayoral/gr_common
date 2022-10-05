import rospy
from manager import Manager
import sys

USAGE="python main.py test_id number_repetitions" 

if __name__ == "__main__":
    rospy.init_node("experiments")
    
    if len(sys.argv) != 3:
        print(USAGE)
        sys.exit()

    manager = Manager(sys.argv[1])

    while not rospy.is_shutdown() and not manager.has_finished:
        manager.run(int(sys.argv[2]))

    rospy.loginfo("experiments finished")
