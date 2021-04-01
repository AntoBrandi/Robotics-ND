#!/usr/bin/env python
'''arm_mover ROS Node'''
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from simple_arm.srv import *


def clamp_at_boundaries(requested_j1, requested_j2):
    '''
    Check if the requested joint angles are whithin the joint boundaries
    If the requested angle is outside the boundaries, set it to the closest boundary
    '''
    clamped_j1 = requested_j1
    clamped_j2 = requested_j2

    min_j1 = rospy.get_param('~min_joint_1_angle', 0)
    max_j1 = rospy.get_param('~max_joint_1_angle', 2*math.pi)
    min_j2 = rospy.get_param('~min_joint_2_angle', 0)
    max_j2 = rospy.get_param('~max_joint_2_angle', 2*math.pi)

    if not min_j1 <= requested_j1 <= max_j1:
        clamped_j1 = min(max(requested_j1, min_j1), max_j1)
        rospy.logwarn('j1 is out of bounds, valid range (%s, %s), clamping to: %s', min_j1, max_j1, clamped_j1)

    if not min_j2 <= requested_j2 <= max_j2:
        clamped_j2 = min(max(requested_j2, min_j2), max_j2)
        rospy.logwarn('j2 is out of bounds, valid range (%s, %s), clamping to: %s', min_j2, max_j2, clamped_j2)

    return clamped_j1, clamped_j2


def at_goal(pos_j1, goal_j1, pos_j2, goal_j2):
    '''
    Check if each joint reaches its goal within a certain tolerance
    '''
    tolerance = 0.05
    result = abs(pos_j1 - goal_j1) <= abs(tolerance)
    result = result and abs(pos_j2 - goal_j2) <= abs(tolerance)
    return result


def move_arm(pos_j1, pos_j2):
    '''
    Send the joint angle command to each joint controller
    and wait until each joint reaches its goal
    '''
    time_elapsed = rospy.Time.now()
    j1_publisher.publish(pos_j1)
    j2_publisher.publish(pos_j2)

    while True:
        joint_state = rospy.wait_for_message('/simple_arm/joint_states', JointState)
        if at_goal(joint_state.position[0], pos_j1, joint_state.position[1], pos_j2):
            time_elapsed = joint_state.header.stamp - time_elapsed
            break

    return time_elapsed


def handle_safe_move_request(req):
    '''Is called every time a new goal request is received from the service'''
    rospy.loginfo('GoToPositionRequest Received - j1:%s, j2:%s', req.joint_1, req.joint_2)
    # check if the requested angles are within the boundaries
    clamp_j1, clamp_j2 = clamp_at_boundaries(req.joint_1, req.joint_2)
    # reach each joint's goal
    time_elapsed = move_arm(clamp_j1, clamp_j2)

    # return a response message to the service caller
    return GoToPositionResponse(time_elapsed)


def mover_service():
    '''Inits the node and the service'''
    rospy.init_node('arm_mover', anonymous=True)
    service = rospy.Service('~safe_move', GoToPosition, handle_safe_move_request)
    rospy.spin()


if __name__ == '__main__':
    j1_publisher = rospy.Publisher('/simple_arm/joint_1_position_controller/command', Float64, queue_size=10)
    j2_publisher = rospy.Publisher('/simple_arm/joint_2_position_controller/command', Float64, queue_size=10)

    try:
        mover_service()
    except rospy.ROSInterruptException:
        pass

