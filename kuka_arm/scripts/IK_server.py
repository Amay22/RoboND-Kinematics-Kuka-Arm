#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
import sympy as sp
import numpy as np
from math import atan2, cos, sin, acos, pi

# DH Parameters
'''
DH Table
Link    a       alpha   d       theta
1       0        0      d1      *
2       a1      90      0       *
3       a2      0       0       *
4       -a3     90      d3      *
5       0       -90     0       *
6       0       90      0       *
EE      0       0       d6      *
'''

a_1 = 0.35
a_2 = 1.25
a_3 = -0.054
d_1 = 0.75
d_3 = 1.5
d_6 = 0.303

# Joint angle symbols
q0, q1, q2, q3, q4, q5, q6 = sp.symbols('q0:7') # theta angles
d0, d1, d2, d3, d4, d5, d6 = sp.symbols('d0:7') # distance between frames (x) along z
a0, a1, a2, a3, a4, a5, a6 = sp.symbols('a0:7') # distance between frames (z) along x
# angle change between zi-1 axis and zi about xi-1
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = sp.symbols('alpha0:7')

# Uncomment for debugging
# def rot_y(angle):
#     '''
#     Rotation about y axis
#     :param angle: magnitude of rotation about y
#     :return: rotation matrix
#     '''
#     return sp.Matrix([[sp.cos(angle), 0, sp.sin(angle), 0],
#                      [0, 1, 0, 0],
#                      [-sp.sin(angle), 0, sp.cos(angle), 0],
#                      [0, 0, 0, 1]])

# def rot_z(angle):
#     '''
#     Rotation about z axis
#     :param angle: amount of rotation about z
#     :return: rotation matrix
#     '''
#     return sp.Matrix([[sp.cos(angle), -sp.sin(angle), 0, 0],
#                      [sp.sin(angle), sp.cos(angle), 0, 0],
#                      [0, 0, 1, 0],
#                      [0, 0, 0, 1]])

def threshold_angle(angle, min_threshold, max_threshold):
    '''
    Check that angles are within rotation bounds
    :param angle: calculated angle
    :param min_threshold: lower bound threshold in degrees
    :param max_threshold: upper bound threshold in degrees
    :return: bounded angle within the treshold
    '''
    if angle > pi * max_threshold / 180:
        return pi * max_threshold / 180
    elif angle < pi * min_threshold / 180:
        return pi * min_threshold / 180
    else:
        return angle

def calculate_hypotenuse(a, b):
    '''
    calculate the hypotenuse using two sides root(a*a + b*b) of a right (90) triangle
    :param a: length of one of the shorter sides
    :param b: length of another of the shorter sides
    :return: length of the hypotenuse (longest) side 
    '''
    return np.sqrt(a*a + b*b)

def calculate_arm_IK(px, py, pz):
    '''
    Calculates the inverse kinematics of the arm section
    :param px: position along the x axis
    :param py: position along the y axis
    :param pz: position along the z axis
    :return: first three arm angles, theta 1, 2 and 3
    '''

    # theta1 Calculate joint angles using Geometric IK method
    theta1 = threshold_angle(atan2(py, px), -185, 185)

    # remove base offsets from the wrist coords for theta 2 and 3
    a_1_x = a_1 * cos(theta1)  # link a_1 offset in x direction
    a_1_y = a_1 * sin(theta1)
    d_6_x = d_6 * cos(theta1)  # link d_6 offset in x direction
    # get the desired end arm x, y, z coordinates
    x_d = px - d_6_x - a_1_x
    y_d = py - a_1_y
    z_d = pz - d_1 + d_6 / 3
    # x y plane arm projections
    r_xy = calculate_hypotenuse(x_d, y_d)
    # x, y, z 3D plane arm length
    r_xyz = np.sqrt(x_d * x_d + y_d * y_d + z_d * z_d)

    # calculate link3 shoulder angle and distance to wrist center
    link_3 = calculate_hypotenuse(a_3, d_3)
    link_3_theta = atan2(a_3, d_3)

    # theta 2
    # link 1 to wrist center angle from z axis / vertical (pi/2)
    beta2 = atan2(r_xy, z_d)
    # law of cosine rule
    D_theta2 = (a_2 * a_2 + r_xyz * r_xyz - link_3 * link_3) / (2 * a_2 * r_xyz)
    psi2 = atan2(np.sqrt(np.abs(1 - D_theta2 ** 2)), D_theta2)
    # zero angle is along z axis
    theta2 = threshold_angle(beta2 - psi2, -45, 85)

    # theta3
    # law of cosine rule
    D_theta3 = (a_2 * a_2 + link_3 * link_3 - r_xyz * r_xyz) / (2 * a_2 * link_3)
    psi3 = atan2(np.sqrt(np.abs(1 - D_theta3 ** 2)), D_theta3) + link_3_theta
    # angle perpendicular wrt link 1 but psi is from link 1
    theta3 = threshold_angle((psi3 - pi / 2) * -1.0, -210, 155 - 90)

    return theta1, theta2, theta3, psi2, beta2, psi3

def calclate_wrist_speherical_IK(px, py, pz, theta1, psi2, beta2, psi3):
    '''
    Calculates the inverse kinematics of the arm section
    :param px: position along the x axis
    :param py: position along the y axis
    :param pz: position along the z axis
    :param theta1: angle of the first rotation point
    :param psi2: angle 2 of the triangle of the arm section from the base
                   to the wrist
    :param beta2: angle between link1 and angle from base to arm wrist point
    :param psi3: angle 3 of the triangle of the arm section from the base
                   to the wrist
    :return: wrist angles, theta 4, 5 and 6
    '''

    # theta 4
    # create a plane onto the shelf with origin in the center and inverted axes
    # rotate wrist around origin so axis 5 is the tangent of the circle from the origin
    y_o = -py
    z_o = (pz - d_1 + a_2) * -1.0  # origin of target shelf
    theta4 =  atan2(z_o, y_o) - pi / 2
    theta4 = threshold_angle(theta4, -350, 350)

    # theta 5
    # keeps wrist level using geometric association laws
    psi5 = pi - psi2 - psi3
    beta5 = pi / 2 - beta2
    theta5 = -(psi5 - beta5)
    if z_o < 0:
        theta5 = -theta5
    if sin(theta1) > 0.55:
        theta5 -= pi / 2 * sin(theta1)

    theta5 = threshold_angle(theta5, -125, 125)

    # theta 6 
    # calculate by thresholding w.r.t. theta4 along ground plane
    theta6 = threshold_angle(-theta4, -350, 350)

    return theta4, theta5, theta6

# Good for Debugging
# def calculate_DH_FK(q0, q1, q2, q3, q4, q5):
#     '''
#     calculates the modified DH forward kinematics
#     :param q0: angle of rotation of joint 1
#     :param q1: angle of rotation of joint 2
#     :param q2: angle of rotation of joint 3
#     :param q3: angle of rotation of joint 4
#     :param q4: angle of rotation of joint 5
#     :param q5: angle of rotation of joint 6
#     :return: Transformation matrix
#     '''

#     s = {a0: 0,     alpha0: 0,          d0: d_1,
#          a1: a_1,   alpha1: pi / 2,     d1: 0,      q1: q1 + pi/2,
#          a2: a_2,   alpha2: 0,          d2: 0,
#          a3: a_3,   alpha3: pi / 2,     d3: d_3,
#          a4: 0,     alpha4: -pi / 2,    d4: 0,
#          a5: 0,     alpha5: pi / 2,     d5: 0,
#          a6: 0,     alpha6: 0,          d6: d_6,    q6: 0}

#     # Modified DH params
#     T0_1 = sp.Matrix([[sp.cos(q0), -sp.sin(q0), 0, a0],
#                    [sp.sin(q0) * sp.cos(alpha0), sp.cos(q0) * sp.cos(alpha0), -sp.sin(alpha0), -sp.sin(alpha0) * d0],
#                    [sp.sin(q0) * sp.sin(alpha0), sp.cos(q0) * sp.sin(alpha0), sp.cos(alpha0), sp.cos(alpha0) * d0],
#                    [0, 0, 0, 1]])
#     T0_1 = T0_1.subs(s)

#     T1_2 = sp.Matrix([[sp.cos(q1), -sp.sin(q1), 0, a1],
#                    [sp.sin(q1) * sp.cos(alpha1), sp.cos(q1) * sp.cos(alpha1), -sp.sin(alpha1), -sp.sin(alpha1) * d1],
#                    [sp.sin(q1) * sp.sin(alpha1), sp.cos(q1) * sp.sin(alpha1), sp.cos(alpha1), sp.cos(alpha1) * d1],
#                    [0, 0, 0, 1]])
#     T1_2 = T1_2.subs(s)

#     T2_3 = sp.Matrix([[sp.cos(q2), -sp.sin(q2), 0, a2],
#                    [sp.sin(q2) * sp.cos(alpha2), sp.cos(q2) * sp.cos(alpha2), -sp.sin(alpha2), -sp.sin(alpha2) * d2],
#                    [sp.sin(q2) * sp.sin(alpha2), sp.cos(q2) * sp.sin(alpha2), sp.cos(alpha2), sp.cos(alpha2) * d2],
#                    [0, 0, 0, 1]])
#     T2_3 = T2_3.subs(s)

#     T3_4 = sp.Matrix([[sp.cos(q3), -sp.sin(q3), 0, a3],
#                    [sp.sin(q3) * sp.cos(alpha3), sp.cos(q3) * sp.cos(alpha3), -sp.sin(alpha3), -sp.sin(alpha3) * d3],
#                    [sp.sin(q3) * sp.sin(alpha3), sp.cos(q3) * sp.sin(alpha3), sp.cos(alpha3), sp.cos(alpha3) * d3],
#                    [0, 0, 0, 1]])
#     T3_4 = T3_4.subs(s)

#     T4_5 = sp.Matrix([[sp.cos(q4), -sp.sin(q4), 0, a4],
#                    [sp.sin(q4) * sp.cos(alpha4), sp.cos(q4) * sp.cos(alpha4), -sp.sin(alpha4), -sp.sin(alpha4) * d4],
#                    [sp.sin(q4) * sp.sin(alpha4), sp.cos(q4) * sp.sin(alpha4), sp.cos(alpha4), sp.cos(alpha4) * d4],
#                    [0, 0, 0, 1]])
#     T4_5 = T4_5.subs(s)

#     T5_6 = sp.Matrix([[sp.cos(q5), -sp.sin(q5), 0, a5],
#                    [sp.sin(q5) * sp.cos(alpha5), sp.cos(q5) * sp.cos(alpha5), -sp.sin(alpha5), -sp.sin(alpha5) * d5],
#                    [sp.sin(q5) * sp.sin(alpha3), sp.cos(q3) * sp.sin(alpha5), sp.cos(alpha5), sp.cos(alpha5) * d5],
#                    [0, 0, 0, 1]])
#     T5_6 = T5_6.subs(s)

#     T6_7 = sp.Matrix([[sp.cos(q6), -sp.sin(q6), 0, a6],
#                    [sp.sin(q6) * sp.cos(alpha6), sp.cos(q6) * sp.cos(alpha6), -sp.sin(alpha6), -sp.sin(alpha6) * d6],
#                    [sp.sin(q6) * sp.sin(alpha6), sp.cos(q6) * sp.sin(alpha6), sp.cos(alpha6), sp.cos(alpha6) * d6],
#                    [0, 0, 0, 1]])
#     T6_7 = T6_7.subs(s)

#     # Define Modified DH Transformation matrix
#     return sp.simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
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

            # Arm inverse kinematics calculation
            theta1, theta2, theta3, alpha2, beta2, alpha3 = calculate_arm_IK(px, py, pz)

            # Spherical wrist calculatino
            theta4, theta5, theta6 = calclate_wrist_speherical_IK(px, py, pz, theta1, alpha2, beta2, alpha3)

            print("t1: {:.3f}   t2: {:.3f}   t3: {:.3f}   t4: {:.3f}   t5: {:.3f}   t6: {:.3f}".
                  format(theta1*180 / pi, theta2*180 / pi, theta3*180 / pi,
                         theta4*180 / pi, theta5*180 / pi, theta6*180 / pi))

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        # code has been commented out as it is not needed for this project but good for debugging
        # T0_6 = calculate_DH_FK(theta1, theta2, theta3, theta4, theta5, theta6)
        # Rrpy_cor = rot_y(pi / 2) * rot_z(pi)
        # E = T0_6 * Rrpy_cor
        # print(T0_6)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
