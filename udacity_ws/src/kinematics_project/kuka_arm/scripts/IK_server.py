#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy.mpmath import *
from sympy import *


### Define functions for Rotation Matrices about x, y, and z given specific angle.
def rot_x(q):
    '''Elementary Rotation Matrix along the X-Axis'''
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(q),  -sin(q)],
                  [ 0,         sin(q),  cos(q)]])
    return R_x
    
def rot_y(q):  
    '''Elementary Rotation Matrix along the Y-Axis'''
    R_y = Matrix([[ cos(q),         0,  sin(q)],
                  [      0,         1,       0],
                  [-sin(q),         0,  cos(q)]])
    return R_y

def rot_z(q):
    '''Elementary Rotation Matrix along the Z-Axis'''
    R_z = Matrix([[ cos(q), -sin(q),        0],
                  [ sin(q),  cos(q),        0],
                  [ 0,              0,      1]])
    return R_z


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta or q
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        s = {alpha0:     0,    a0:      0,   d1:  0.75,
             alpha1: -pi/2,    a1:   0.35,   d2:     0,    q2: q2-pi/2,
             alpha2:     0,    a2:   1.25,   d3:     0,
             alpha3: -pi/2,    a3: -0.054,   d4:  1.50,
             alpha4:  pi/2,    a4:      0,   d5:     0,
             alpha5: -pi/2,    a5:      0,   d6:     0,
             alpha6:     0,    a6:      0,   d7: 0.303,    q7:       0}

	    # base_link to link_1
        T0_1 = Matrix([[            cos(q1),              -sin(q1),                0,                 a0],
                       [sin(q1)*cos(alpha0),    cos(q1)*cos(alpha0),    -sin(alpha0),    -sin(alpha0)*d1],
                       [sin(q1)*sin(alpha0),    cos(q1)*sin(alpha0),     cos(alpha0),     cos(alpha0)*d1],
                       [                  0,                      0,               0,                  1]])

        # link_1 to link_2
        T1_2 = Matrix([[            cos(q2),              -sin(q2),                0,                 a1],
                       [sin(q2)*cos(alpha1),    cos(q2)*cos(alpha1),    -sin(alpha1),    -sin(alpha1)*d2],
                       [sin(q2)*sin(alpha1),    cos(q2)*sin(alpha1),     cos(alpha1),     cos(alpha1)*d2],
                       [                  0,                      0,               0,                  1]])

        # link_2 to link_3
        T2_3 = Matrix([[            cos(q3),              -sin(q3),                0,                 a2],
                       [sin(q3)*cos(alpha2),    cos(q3)*cos(alpha2),    -sin(alpha2),    -sin(alpha2)*d3],
                       [sin(q3)*sin(alpha2),    cos(q3)*sin(alpha2),     cos(alpha2),     cos(alpha2)*d3],
                       [                  0,                      0,               0,                  1]])

        # link_3 to link_4
        T3_4 = Matrix([[            cos(q4),              -sin(q4),                0,                 a3],
                       [sin(q4)*cos(alpha3),    cos(q4)*cos(alpha3),    -sin(alpha3),    -sin(alpha3)*d4],
                       [sin(q4)*sin(alpha3),    cos(q4)*sin(alpha3),     cos(alpha3),     cos(alpha3)*d4],
                       [                  0,                      0,               0,                  1]])

        # link_4 to link_5
        T4_5 = Matrix([[            cos(q5),              -sin(q5),                0,                 a4],
                       [sin(q5)*cos(alpha4),    cos(q5)*cos(alpha4),    -sin(alpha4),    -sin(alpha4)*d5],
                       [sin(q5)*sin(alpha4),    cos(q5)*sin(alpha4),     cos(alpha4),     cos(alpha4)*d5],
                       [                  0,                      0,               0,                  1]])

        # link_5 to link_6
        T5_6 = Matrix([[            cos(q6),              -sin(q6),                0,                 a5],
                       [sin(q6)*cos(alpha5),    cos(q6)*cos(alpha5),    -sin(alpha5),    -sin(alpha5)*d6],
                       [sin(q6)*sin(alpha5),    cos(q6)*sin(alpha5),     cos(alpha5),     cos(alpha5)*d6],
                       [                  0,                      0,               0,                  1]])

	    # Extract rotation matrices 
        T0_1 = T0_1.subs(s)
        T1_2 = T1_2.subs(s)
        T2_3 = T2_3.subs(s)
        T3_4 = T3_4.subs(s)
        T4_5 = T4_5.subs(s)
        T5_6 = T5_6.subs(s)
        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]

        # Compensate the Rotation between URDF and TF final link
        R_adj = rot_z(radians(180)) * rot_y(radians(-90))

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_adj

            # Calculate joint angles using Geometric IK method
            # Position of the Wirst Center
            EE = Matrix([[px],
                         [py],
                         [pz]])
            WC = EE - 0.303 * Rrpy[:,2]
            Wx = WC[0]
            Wy = WC[1]
            Wz = WC[2]
            
            # First Three joint angles - Position Problem
            theta1 = atan2(Wy, Wx)

            side_a = 1.501
            side_b = sqrt(pow((sqrt(Wx*Wx + Wy*Wy) -0.35), 2) + pow((Wz - 0.75), 2))
            side_c = 1.25

            angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a)/(2 * side_b * side_c))
            angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b)/(2 * side_a * side_c))
            angle_a = acos((side_a*side_a + side_b*side_b - side_c*side_c)/(2 * side_a * side_b))

            theta2 = pi / 2 - angle_a - atan2(Wz-0.75, sqrt(Wx*Wx + Wy*Wy) - 0.35)
            theta3 = pi / 2 - (angle_b + 0.036)

            # Last Three joint angles - Orientation Problem
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv(method="LU") * Rrpy

            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
