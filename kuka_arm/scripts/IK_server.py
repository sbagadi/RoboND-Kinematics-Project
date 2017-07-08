#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
from optparse import OptionParser
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

# Define DH param symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # Theta_i

# Modified DH params
s = {alpha0:      0, a0:      0, d1:  0.75,
     alpha1:  -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
     alpha2:      0, a2:   1.25, d3:     0,
     alpha3:  -pi/2, a3: -0.054, d4:   1.5,
     alpha4:   pi/2, a4:      0, d5:     0,
     alpha5:  -pi/2, a5:      0, d6:     0,
     alpha6:      0, a6:      0, d7: 0.303, q7: 0}

# Define Modified DH Transformation matrix

# Create individual transformation matrices
# base_link to link_1
T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
               [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                  0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)

# link_1 to link_2
T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
               [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                  0,                   0,            0,               1]])
T1_2 = T1_2.subs(s)

# link_2 to link_3
T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
               [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                  0,                   0,            0,               1]])
T2_3 = T2_3.subs(s)

# link_3 to link_4
T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
               [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                  0,                   0,            0,               1]])
T3_4 = T3_4.subs(s)

# link_4 to link_5
T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
               [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                  0,                   0,            0,               1]])
T4_5 = T4_5.subs(s)

# link_5 to link_6
T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
               [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                  0,                   0,            0,               1]])
T5_6 = T5_6.subs(s)

# link_6 to gripper
T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
               [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                  0,                   0,            0,               1]])
T6_G = T6_G.subs(s)

# # Composition of Homogeneous Transforms.
T0_2 = simplify(T0_1 * T1_2)  # base_link to link_2
T0_3 = simplify(T0_2 * T2_3)  # base_link to link_3
T0_4 = simplify(T0_3 * T3_4)  # base_link to link_4
T0_5 = simplify(T0_4 * T4_5)  # base_link to link_5
T0_6 = simplify(T0_5 * T5_6)  # base_link to link_6
T0_G = simplify(T0_6 * T6_G)  # base_link to gripper

# Correction needed to account for the orientation difference between definition of Gripper Link in URDF
# versus DH convention.
R_z = Matrix([[    cos(pi), -sin(pi),             0,      0],
              [    sin(pi),  cos(pi),             0,      0],
              [          0,        0,             1,      0],
              [          0,        0,             0,      1]])

R_y = Matrix([[ cos(-pi/2),        0,    sin(-pi/2),      0],
              [          0,        1,             0,      0],
              [-sin(-pi/2),        0,    cos(-pi/2),      0],
              [          0,        0,             0,      1]])
R_corr = simplify(R_z * R_y)

# Total homogeneous transform between base_link and gripper_link with orientation correction applied.
T_total = simplify(T0_G * R_corr)


def evaluate_forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    total = T_total.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    return total[0, 3], total[1, 3], total[2, 3]


R, P, Y = symbols('R, P, Y')
R_x = Matrix([[1,      0,       0],
              [0, cos(R), -sin(R)],
              [0, sin(R),  cos(R)]])
R_y = Matrix([[ cos(P), 0, sin(P)],
              [      0, 1,      0],
              [-sin(P), 0, cos(P)]])
R_z = Matrix([[cos(Y), -sin(Y), 0],
              [sin(Y),  cos(Y), 0],
              [     0,       0, 1]])

Rxyz = simplify(R_x * R_y * R_z)
Rzyx = simplify(R_z * R_y * R_x)

def generate_data():
    list = []
    list.append([2.04300328191, -3.27098214715e-07, 1.94599164004,
                 1.58570096609e-07, 5.14022570464e-06, -1.60106544558e-07])
    list.append([1.51315888411, 0.912443637483, 2.86997051454,
                 -0.126707965534, -0.772660127748, 0.458727422927])
    list.append([1.77226567827, 1.31854834519, 2.35869234617,
                 0.0360590763007, -0.659842294056, 0.353075723536])
    list.append([2.05909508656, 1.23706120462, 1.67084618882,
                 0.0490620094169, -0.36734636027, 0.218193333796])
    list.append([2.09002597191, 0.900040987518, 0.811100065526,
                -0.000827238027711, 0.00091896481172, -0.000362364530142])
    list.append([1.6218, 1.0705, 1.8786, 0.493234, 0.187709, 0.163823])
    list.append([2.15286, 0, 1.94653, 0, -0.000148353, 0])
    return list


def debug_calculate_IK():
    # Values from a sample run.
    # (px, py, pz) = (2.04300328191, -3.27098214715e-07, 1.94599164004)
    # (roll, pitch, yaw) = (1.58570096609e-07, 5.14022570464e-06, -1.60106544558e-07)
    #
    # (px, py, pz) = (1.51315888411, 0.912443637483, 2.86997051454)
    # (roll, pitch, yaw) = (-0.126707965534, -0.772660127748, 0.458727422927)
    #
    # (px, py, pz) = (1.77226567827, 1.31854834519, 2.35869234617)
    # (roll, pitch, yaw) = (0.0360590763007, -0.659842294056, 0.353075723536)
    #
    # (px, py, pz) = (2.05909508656, 1.23706120462, 1.67084618882)
    # (roll, pitch, yaw) = (0.0490620094169, -0.36734636027, 0.218193333796)
    #
    # (px, py, pz) = (2.09269804161, 1.19551262932, 1.51926655903)
    # (roll, pitch, yaw) = (0.0467742480674, -0.304139190236, 0.185314400318)
    #
    # (px, py, pz) = (2.11499842934, 1.14700373924, 1.36969162121)
    # (roll, pitch, yaw) = (0.0423014903548, -0.241505213195, 0.151140904718)
    #
    # (px, py, pz) = (2.1258525072, 1.09234531968, 1.22312928294)
    # (roll, pitch, yaw) = (0.0354791245817, -0.179555422553, 0.115589602507)
    #
    # (px, py, pz) = (2.09002597191, 0.900040987518, 0.811100065526)
    # (roll, pitch, yaw) = (-0.000827238027711, 0.00091896481172, -0.000362364530142)
    positions = generate_data()
    for position in positions:
        px = position[0]
        py = position[1]
        pz = position[2]
        roll = position[3]
        pitch = position[4]
        yaw = position[5]
        theta1, theta2, theta3, theta4, theta5, theta6 = calculate_IK(px, py, pz, roll, pitch, yaw, True)
        forward_pos = evaluate_forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6)
        print 'Input: (px, py, pz) = (', px, ', ', py, ', ', pz, '), Output: (px, py, pz) =', forward_pos


def calculate_IK(px, py, pz, roll, pitch, yaw, debug=False):
    # Get rotation matrix from gripper pose.
    R0_G = Rxyz.evalf(subs={R: roll, P: pitch, Y: yaw})

    # Calculate Rrpy.
    R_corr = R_z * R_y
    R_corr = R_corr.evalf(subs={P: -(pi/2), Y: pi})
    Rrpy = R0_G[0:3, 0:3] * R_corr

    # Get wrist center position.
    wx = px - (s[d6] + s[d7]) * R0_G[0, 0]
    wy = py - (s[d6] + s[d7]) * R0_G[1, 0]
    wz = pz - (s[d6] + s[d7]) * R0_G[2, 0]

    # theta1 can be easily calculated by rotating the first joint towards the wrist center. Imagine a vector from origin
    # of the base frame to the projection of the wrist center on the x-y plane. The rotation angle is the angle between
    # this vector and the x-axis.
    theta1 = atan2(wy, wx)

    # Find the origin of the second link using theta1
    t0_2 = T0_2.evalf(subs={q1: theta1})
    o2 = [t0_2[0, 3], t0_2[1, 3], t0_2[2, 3]]

    l2 = sqrt((o2[0] - wx)**2 + (o2[1] - wy)**2 + (o2[2] - wz)**2)
    l3 = sqrt(s[a3]**2 + s[d4]**2)

    # Lets take a triangle of with sides a, b, c and angles alpha (opposite to a), beta (opposite to b) and gamma
    # (opposite to c).
    #
    # Using cosine rule, we get:
    # cos(alpha) = (b**2 + c**2 - a**2) / (2 * b * c) := D
    # alpha = cos^-1(D)
    # sin(alpha) = -/+ sqrt(1 - cos(alpha)**2) => sin(alpha) = -/+ sqrt(1 - D**2)
    #
    # we can now use atan2 to get the angle alpha with the correct sign
    # alpha = atan2(sin(alpha), cos(alpha) = atan2(sqrt(1 - D**2), D)
    #
    # Source: http://smpp.northwestern.edu/savedLiterature/Spong_Textbook.pdf
    #
    # The phi angle is formed by a2 and l3 with l2 as the opposite side of the triangle.
    # Here we are actually interested in the complimentary angle (alpha = 1 - phi) so we use cos(alpha) = -cos(phi)
    D1 = (l2**2 - s[a2]**2 - l3**2) / (2 * s[a2] * l3)
    D1 = np.clip(D1, None, 1.0)  # Avoids getting imaginary numbers.

    alpha = atan2(sqrt(1 - D1**2), D1)

    # This is to account for the offset between joint 3 and joint 4 given by a2.
    beta = atan2(s[a3], s[d4])

    theta3 = (alpha + beta) - (pi / 2)

    # Similar to D1 calculations.
    # the gamma angle is formed by the sides a2 and l2 with l3 as the opposite side).
    D2 = (s[a2]**2 + l2**2 - l3**2) / (2 * s[a2] * l2)
    D2 = np.clip(D2, None, 1.0)  # Avoids getting imaginary numbers.

    gamma = atan2((wz - o2[2]), sqrt((o2[0] - wx)**2 + (o2[1] - wy)**2))
    delta = atan2(sqrt(1 - D2**2), D2)

    theta2 = (pi / 2) - (gamma + delta)

    R0_3 = T0_3[0:3, 0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    R3_6 = R0_3.transpose()[0:3, 0:3] * Rrpy[0:3, 0:3]

    # R3_6 = Matrix([
    #     [-s(q4) * s(q6) + c(q4) * c(q5) * c(q6),    -s(q4) * c(q6) - s(q6) * c(q4) * c(q5),    -s(q5) * c(q4)],
    #     [                         s(q5) * c(q6),                            -s(q5) * s(q6),             c(q5)],
    #     [-s(q4) * c(q5) * c(q6) - s(q6) * c(q4),     s(q4) * s(q6) * c(q5) - c(q4) * c(q6),     s(q4) * s(q5)]])
    #
    # theta6 = atan2(-r22, r21)
    # theta5 = atan2(sqrt(r13**2 + r33**2), r23)
    # theta6 = atan2(r33, -r13)

    theta6 = atan2(-R3_6[1, 1], R3_6[2, 1])
    theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])

    print '(theta1, theta2, theta3, theta4, theta5, theta6) = (', \
        theta1, ', ', theta2, ', ',  theta3, ', ', theta4, ', ', theta5, ', ', theta6, ')'
    return theta1, theta2, theta3, theta4, theta5, theta6

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        total_error = 0
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

            # Calculate joint angles using Geometric IK method.
            # Step 1: Calculate Wrist center.

            theta1, theta2, theta3, theta4, theta5, theta6 = calculate_IK(px, py, pz, roll, pitch, yaw)
            px_f, py_f, pz_f = evaluate_forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6)
            total_error += (px_f - px)**2 + (py_f - py)**2 + (pz_f - pz)**2

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        mean_square_error = total_error / len(req.poses)
        print 'Mean square error = ', mean_square_error
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("-d", action="store_true", dest="run_debug", default=False)
    (options, args) = parser.parse_args()
    if options.run_debug:
        print 'Starting in debug mode'
        debug_calculate_IK()
    else:
        IK_server()