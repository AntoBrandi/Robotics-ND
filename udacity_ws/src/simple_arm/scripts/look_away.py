#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Image, JointState
from simple_arm.srv import *


class LookAway(object):
    '''
    When a uniform color pattern is viewed from the camera, move the robot arm away 
    '''
    def __init__(self):
        rospy.init_node('look_away')

        # Before init the nodes make sure all the modules are started
        rospy.wait_for_message('/simple_arm/joint_states', JointState)
        rospy.wait_for_message('/rgb_camera/image_raw', Image)

        self.sub1 = rospy.Subscriber('/simple_arm/joint_states', JointState, self.joint_states_callback)
        self.sub2 = rospy.Subscriber('/rgb_camera/image_raw', Image, self.look_away_callback)
        self.safe_move = rospy.ServiceProxy('/arm_mover/safe_move', GoToPosition)

        self.last_position = None
        self.arm_moving = False

        rospy.spin()


    def uniform_image(self, image):
        '''
        HELPER FUNCTION
        Checks if there is an uniform pattern in the image
        meaning that checks if all the pixel values in the image ar the same 
        '''
        return all(value == image[0] for value in image)


    def coord_equal(self, coord_1, coord_2):
        '''
        HELPER FUNCTION
        Receives a pair of joint coordinates and checks if are euqals 
        or at least within a certain boundary.
        Return a boolean whether or not the two sets of coordinates are equals
        '''
        if coord_1 is None or coord_2 is None:
            return False
        tolerance = .0005
        result = abs(coord_1[0] - coord_2[0]) <= abs(tolerance)
        result = result and abs(coord_1[1] - coord_2[1]) <= abs(tolerance)
        return result


    def joint_states_callback(self, data):
        '''
        Called every time a new message is published in the topic /simple_arm/joint_states
        Checks if the robot arm is moving or not by comparing the current joint angles with the previous
        '''
        if self.coord_equal(data.position, self.last_position):
            self.arm_moving = False
        else:
            self.last_position = data.position
            self.arm_moving = True


    def look_away_callback(self, data):
        '''
        Called every time a new message is published in the topic /rgb_camera/image_raw
        Check if the camera of the robot is looking to the sky. If so, and the robot is
        in rest state, then move the robot to make it look away
        '''
        if not self.arm_moving and self.uniform_image(data.data):
            try:
                # use the safe_mover service implemented in the arm_mover node to 
                # request and apply a robot mouvement
                rospy.wait_for_service('/arm_mover/safe_move')
                msg = GoToPositionRequest()
                msg.joint_1 = 1.57
                msg.joint_2 = 1.57
                response = self.safe_move(msg)

                rospy.logwarn("Camera detecting uniform image. \
                               Elapsed time to look at something nicer:\n%s", 
                               response)

            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: %s", e)


if __name__ == '__main__':
    try: 
        LookAway()
    except rospy.ROSInterruptException:
        pass