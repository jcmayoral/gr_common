#!/usr/bin python
import rospy
import actionlib
from gr_action_msgs.msg import SimMotionPlannerAction, SimMotionPlannerGoal
from patterns import Rectangle2D
from smach_ros import SimpleActionState, IntrospectionServer
from smach_ros import ConditionState
import smach
import tf

def motion_goalcb(userdata, goal):
    #NOTE Start and goal must have same orientation
    #goal = SimMotionPlannerAction()
    start = userdata.start
    end = userdata.end
    #start = None
    #end = None
    print(start, end)

    goal.setstart = False

    if start is None:
        return goal

    goal.setstart = True
    goal.is_motion = False

    goal.startpose.pose.position.x = start[0]
    goal.startpose.pose.position.x = start[1]
    quaternion = tf.transformations.quaternion_from_euler(0, 0, start[2])
    #type(pose) = geometry_msgs.msg.Pose
    goal.startpose.pose.orientation.x = quaternion[0]
    goal.startpose.pose.orientation.y = quaternion[1]
    goal.startpose.pose.orientation.z = quaternion[2]
    goal.startpose.pose.orientation.w = quaternion[3]
    print(goal)

    return goal

def motion_resultcb(userdata, status, result):
    #if status == GoalStatus.SUCCEEDED:
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

    pattern = Rectangle2D(id="rectangle", height=10, width=10, center=(-10,0,0), outcomes = ['succeeded', 'aborted'], output_keys=['start', 'end'])

    sm = smach.StateMachine(outcomes = ['succeeded','aborted','preempted'])
    sm.userdata.nums = range(25, 88, 3)
    sm.userdata.even_nums = []
    sm.userdata.odd_nums = []

    with sm:
        sm_it = smach.Iterator(outcomes = ['succeeded','preempted','aborted'],
                               input_keys = ['nums', 'even_nums', 'odd_nums'],
                               it = lambda: range(0, len(sm.userdata.nums)),
                               output_keys = ['even_nums', 'odd_nums'],
                               it_label = 'index',
                               exhausted_outcome = 'succeeded')

        with sm_it:
            sq = smach.Sequence( outcomes = ['succeeded','aborted','preempted'],
                             connector_outcome = 'succeeded')
            with sq:
                smach.Sequence.add('SET_PATTERN', pattern)#, output_keys=['start', 'end'])
                smach.Sequence.add('TRIGGER_MOTION',
                               SimpleActionState("/SimMotionPlanner/my_person",
                                                 SimMotionPlannerAction,
                                                 goal_cb=motion_goalcb,
                                                 result_cb = motion_resultcb,
                                                 server_wait_timeout = rospy.Duration(1000.0),
                                                 input_keys=['start', 'end', 'nums']))
            smach.Iterator.set_contained_state('TRIGGER_MOTION',
                                         sq,
                                         loop_outcomes=['succeeded'])
        smach.StateMachine.add('TUTORIAL_IT',sm_it,
                     {'succeeded':'succeeded',
                      'aborted':'aborted'})

    #sis = IntrospectionServer('experiment_sm_viewer', sm, '/ROOT')
    #sis.start()
    outcome = sm.execute()
    rospy.spin()
    #sis.stop()
