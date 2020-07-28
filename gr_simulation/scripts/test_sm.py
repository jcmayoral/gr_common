#!/usr/bin python
import rospy
import actionlib
from gr_action_msgs.msg import SimMotionPlannerAction, SimMotionPlannerGoal
from patterns import Rectangle2D
from smach_ros import SimpleActionState, IntrospectionServer
import smach

def motion_goalcb(userdata, goal):
    #NOTE Start and goal must have same orientation
    goal = SimMotionPlannerAction()
    print ("GOAL")
    goal.position.x = 2.0
    return goal

def motion_resultcb(userdata, status, result):
    if status == GoalStatus.SUCCEEDED:
        return 'succeeded'

if __name__ == '__main__':

    rospy.init_node("experiment_sm")
    """
    sm = smach.StateMachine(['succeeded','aborted','preempted'])
    sm.pattern = Rectangle2D(id="rectangle", height=10, width=10, center=(0,0,0))
    with sm:
        smach.StateMachine.add('TRIGGER_MOTION',
                           SimpleActionState('/SimMotionPlanner/my_person',
                                             SimMotionPlannerAction,
                                             goal_cb=motion_goalcb,
                                             result_cb = motion_resultcb,
                                             input_keys=['pattern']),
                                transitions={'succeeded':'aborted', 'aborted':'aborted'})
    outcome = sm.execute()
    """
    sq = smach.Sequence( outcomes = ['succeeded','aborted','preempted'],
                         connector_outcome = 'succeeded')
    sq.pattern = Rectangle2D(id="rectangle", height=10, width=10, center=(-10,0,0))
    with sq:
        smach.Sequence.add('TRIGGER_MOTION',
                           SimpleActionState('/SimMotionPlanner/my_person',
                                             SimMotionPlannerAction,
                                             goal_cb=motion_goalcb,
                                             result_cb = motion_resultcb,
                                             server_wait_timeout = rospy.Duration(30.0),
                                             input_keys=['pattern']))

    sis = IntrospectionServer('experiment_sm_viewer', sq, '/ROOT')
    sis.start()
    outcome = sq.execute()
    rospy.spin()
    sis.stop()
