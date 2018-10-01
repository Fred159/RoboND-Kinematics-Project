#!/usr/bin/env python
# Python 2.7

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Authoer: MING LIN

# import realated modules
import rospy

import tf

from kuka_arm.srv import *

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from geometry_msgs.msg import Pose

from mpmath import *

from sympy import *


# define roatation matrix in x, y, z axis (degree 'q' in radian)
def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])
    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])
    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])
    return R_z


# define translation matrix with alpha, a, d, theta(q)
def trans_matrix(alpha, a, d, q):
    T = Matrix([[cos(q), -sin(q), 0, a],
                [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                [0, 0, 0, 1]])
    return T


# define parameter degree to radian coefficient.
d2r = pi / 180.


# define a function calculate the theta1, theta2, theta3 with geometric method.
# follow the udacity guide video
def cal_theta_123(R_EE, px, py, pz, roll, pitch, yaw):
    # Compensate for rotation discrepancy between DH parameters and Gazebo enviroment
    Rot_err = rot_z(180 * d2r) * rot_y(-90 * d2r)

    # multiply Rotation_End-Effector with R_Error
    R_EE = R_EE * Rot_err

    R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    # Wrist center anlge is calculate as below
    WC = Matrix([[px], [py], [pz]]) - (0.303) * R_EE[:, 2]

    # Calculate joint angles using Geometric IK method

    theta1 = atan2(WC[1], WC[0])

    #calculate the theta1,2,3 based on geometry.
    a = 1.5
    b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
    c = 1.25  # Length of joint 1 to 2.
    alpha = acos((b * b + c * c - a * a) / (2 * b * c))
    beta = acos((a * a + c * c - b * b) / (2 * a * c))
    delta = atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
    theta2 = pi / 2 - alpha - delta
    theta3 = pi / 2 - (beta + 0.036)
    return (R_EE, WC, theta1, theta2, theta3)


# fill in the handle_calculate_inverse_kinematic with request from simulator.
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print
        "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols for use
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create DH parameters, follow the udacity guide video.
        s = {alpha0: 0, a0: 0, d1: 0.75, q1: q1,
             alpha1: -90 * d2r, a1: 0.35, d2: 0, q2: q2 - (90 * d2r),
             alpha2: 0, a2: 1.25, d3: 0, q3: q3,
             alpha3: -90 * d2r, a3: -0.054, d4: 1.50, q4: q4,
             alpha4: 90 * d2r, a4: 0, d5: 0, q5: q5,
             alpha5: -90 * d2r, a5: 0, d6: 0, q6: q6,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0
             }

        # Create individual transformation matrices. From coordinate 0 to 1, 1 to 2, 2 to 3, 3 to 4,
        # 4 to 5, 5 to 6, 6 to end effector.
        # Translation matrix include [ [Rotation matrix, Position],[0, 0, 0, 1]] * [[origin coordinate position],[1]]
        T0_1 = trans_matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = trans_matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = trans_matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = trans_matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = trans_matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = trans_matrix(alpha5, a5, d6, q6).subs(s)
        T6_EE = trans_matrix(alpha6, a6, d7, q7).subs(s)

        # Use simplify function calculate translation between 0 coordinate to end-effector coordinate
        T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)

        # Initialize service response
        # Create a joint_trajectory_list to give input to function. Based on this joint_trajectory_point,
        # each joint calculates the theta angle based on the inverse kinematic.
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
            # define symbol here.
            r, p, y = symbols('r p y')

            # Substitute r,p,y to rotation matrix
            R_x = rot_x(r)
            R_y = rot_y(p)
            R_z = rot_z(y)

            # Get rotation matrix of gripper. This rotation define by fix coordinate xyz order.
            R_EE = R_z * R_y * R_x
            # Use cal_theta_123 function to get theta1,2,3
            R_EE, WC, theta1, theta2, theta3 = cal_theta_123(R_EE, px, py, pz, roll, pitch, yaw)

            # Use inverse rotation method to get theta6,5,4
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # Follow the udacity guide.
            # https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/0cf1a5da-3ee4-49f6-8cd2-0d2fc157e11b
            # Privious rotation = inv(Rotation) * origin rotation
            R3_6 = R0_3.inv("LU") * R_EE

            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
            sq5 = -R3_6[1, 1] / sin(theta6)
            cq5 = R3_6[1, 2]
            theta5 = atan2(sq5, cq5)
            sq4 = R3_6[2, 2] / sin(theta5)
            cq4 = -R3_6[0, 2] / sin(theta5)
            theta4 = atan2(sq4, cq4)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            # Add theta1-6 to joint_trajectory_list
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print
    "Ready to receive an IK request!"
    rospy.spin()


if __name__ == "__main__":
    IK_server()

