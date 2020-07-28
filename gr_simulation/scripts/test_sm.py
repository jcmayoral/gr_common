#!/usr/bin python
import rospy
import actionlib
from gr_action_msgs.msg import SimMotionPlannerAction, SimMotionPlannerGoal
from patterns import Rectangle2D
from smach_ros import SimpleActionState, IntrospectionServer
from smach_ros import ConditionState
import smach
import tf
import math

def motion_goalcb(userdata, goal):
    #NOTE Start and goal must have same orientation
    #goal = SimMotionPlannerAction()
    start = userdata.start
    end = userdata.end
    #start = None
    #end = None
    print(start, end)

    fakeyaw = math.atan2(end[1]-start[1],end[0]-start[0])
    print(fakeyaw)
    #start[2] = fakeyaw
    #end[2] = fakeyaw


    goal.setstart = False

    if start is None:
        return goal

    goal.setstart = True
    goal.is_motion = True

    goal.startpose.pose.position.x = start[0]
    goal.startpose.pose.position.y = start[1]
    quaternion = tf.transformations.quaternion_from_euler(0, 0, fakeyaw)
    #type(pose) = geometry_msgs.msg.Pose
    goal.startpose.pose.orientation.x = quaternion[0]
    goal.startpose.pose.orientation.y = quaternion[1]
    goal.startpose.pose.orientation.z = quaternion[2]
    goal.startpose.pose.orientation.w = quaternion[3]


    quaternion = (
    goal.startpose.pose.orientation.x,
    goal.startpose.pose.orientation.y,
    goal.startpose.pose.orientation.z,
    goal.startpose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print(euler)



    goal.goalPose.pose.position.x = end[0]
    goal.goalPose.pose.position.y = end[1]
    quaternion2 = tf.transformations.quaternion_from_euler(0, 0, fakeyaw)
    #type(pose) = geometry_msgs.msg.Pose
    goal.goalPose.pose.orientation.x = quaternion2[0]
    goal.goalPose.pose.orientation.y = quaternion2[1]
    goal.goalPose.pose.orientation.z = quaternion2[2]
    goal.goalPose.pose.orientation.w = quaternion2[3]


    quaternion = (
    goal.goalPose.pose.orientation.x,
    goal.goalPose.pose.orientation.y,
    goal.goalPose.pose.orientation.z,
    goal.goalPose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print(euler)

    print(math.atan2(end[1]-start[1],end[0]-start[0]))
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

    pattern = Rectangle2D(id="rectangle", height=10, width=10, center=(10,0,0), outcomes = ['succeeded', 'aborted'], output_keys=['start', 'end'])

    sm = smach.StateMachine(outcomes = ['succeeded','aborted','preempted'])
    sm.userdata.nums = range(25, 30, 3)

    with sm:
        sm_it = smach.Iterator(outcomes = ['succeeded','preempted','aborted'],
                               input_keys = ['nums', 'even_nums', 'odd_nums'],
                               it = range(6),#lambda: range(0, len(sm.userdata.nums)),
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

    sis = IntrospectionServer('experiment_sm_viewer', sm, '/ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
