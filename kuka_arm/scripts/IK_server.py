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
from forward_kinematics import evaluate_forward_kinematics

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
# T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
#                [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
#                [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
#                [                  0,                   0,            0,               1]])
# T3_4 = T3_4.subs(s)

# link_4 to link_5
# T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
#                [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
#                [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
#                [                  0,                   0,            0,               1]])
# T4_5 = T4_5.subs(s)

# # link_5 to link_6
# T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
#                [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
#                [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
#                [                  0,                   0,            0,               1]])
# T5_6 = T5_6.subs(s)

# link_6 to gripper
# T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
#                [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
#                [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
#                [                  0,                   0,            0,               1]])
# T6_G = T6_G.subs(s)

# # Composition of Homogeneous Transforms.
T0_2 = simplify(T0_1 * T1_2)  # base_link to link_2
T0_3 = simplify(T0_2 * T2_3)  # base_link to link_3
# T0_4 = simplify(T0_3 * T3_4)  # base_link to link_4
# T0_5 = simplify(T0_4 * T4_5)  # base_link to link_5
# T0_6 = simplify(T0_5 * T5_6)  # base_link to link_6
# T0_G = simplify(T0_6 * T6_G)  # base_link to gripper
#
# # Correction needed to account for the orientation difference between definition of Gripper Link in URDF
# # versus DH convention.
# R_z = Matrix([[    cos(pi), -sin(pi),             0,      0],
#               [    sin(pi),  cos(pi),             0,      0],
#               [          0,        0,             1,      0],
#               [          0,        0,             0,      1]])
#
# R_y = Matrix([[ cos(-pi/2),        0,    sin(-pi/2),      0],
#               [          0,        1,             0,      0],
#               [-sin(-pi/2),        0,    cos(-pi/2),      0],
#               [          0,        0,             0,      1]])
# R_corr = simplify(R_z * R_y)
#
# # Total homogeneous transform between base_link and gripper_link with orientation correction applied.
# T_total = simplify(T0_G * R_corr)

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
    # (px, py, pz) = (2.01707456364, 0.206786727217, 2.13930989103)
    # (roll, pitch, yaw) = (-0.0358968005668, -0.149389381877, 0.100348787505)
    #
    # (px, py, pz) = (1.94879772985, 0.403027451294, 2.33051932868)
    # (roll, pitch, yaw) = (-0.0701929090143, -0.298480433717, 0.196435943183)
    #
    # (px, py, pz) = (1.85639474105, 0.568706363488, 2.49474496292)
    # (roll, pitch, yaw) = (-0.0971882896625, -0.43286506654, 0.279815103615)
    #
    # (px, py, pz) = (1.73565152023, 0.713782144824, 2.65351394562)
    # (roll, pitch, yaw) = (-0.119052081093, -0.566288213664, 0.357082154036)
    #
    # (px, py, pz) = (1.63057859711, 0.821373100449, 2.76401707791)
    # (roll, pitch, yaw) = (-0.127213525882, -0.670341351521, 0.412413169628)
    #
    # (px, py, pz) = (1.51315888411, 0.912443637483, 2.86997051454)
    # (roll, pitch, yaw) = (-0.126707965534, -0.772660127748, 0.458727422927)
    #
    # (px, py, pz) = (1.45114409588, 0.98102041606, 2.91309009446)
    # (roll, pitch, yaw) = (-0.111140687682, -0.831302355611, 0.477510854162)
    #
    # (px, py, pz) = (1.38541611224, 1.04444274292, 2.95473314203)
    # (roll, pitch, yaw) = (-0.0873502662687, -0.887647564837, 0.487494286333)
    #
    # (px, py, pz) = (1.40426313865, 1.10323425399, 2.91834336549)
    # (roll, pitch, yaw) = (-0.0592894232713, -0.887510387467, 0.475139842241)
    #
    # (px, py, pz) = (1.42060509425, 1.16245791548, 2.88045210147)
    # (roll, pitch, yaw) = (-0.0304672944938, -0.885711965091, 0.460508801226)
    #
    # (px, py, pz) = (1.49797216739, 1.21672871831, 2.77963707044)
    # (roll, pitch, yaw) = (-0.00808505227664, -0.842964174902, 0.43680416131)
    #
    # (px, py, pz) = (1.57039463146, 1.26605017558, 2.67401765164)
    # (roll, pitch, yaw) = (0.0117628856159, -0.799506075005, 0.413169362707)
    #
    # (px, py, pz) = (1.67625571185, 1.29860392369, 2.5195356699)
    # (roll, pitch, yaw) = (0.0251053285112, -0.729715836457, 0.383612774713)
    #
    # (px, py, pz) = (1.77226567827, 1.31854834519, 2.35869234617)
    # (roll, pitch, yaw) = (0.0360590763007, -0.659842294056, 0.353075723536)
    #
    # (px, py, pz) = (1.87344373408, 1.31543640121, 2.16930787697)
    # (roll, pitch, yaw) = (0.0431067398582, -0.57729667112, 0.317614613976)
    #
    # (px, py, pz) = (1.95911623367, 1.29627096211, 1.97588296912)
    # (roll, pitch, yaw) = (0.0477222917309, -0.495082652544, 0.280326505243)
    #
    # (px, py, pz) = (2.01444594732, 1.27088402825, 1.82340006358)
    # (roll, pitch, yaw) = (0.0493251784281, -0.431025161595, 0.249846400461)
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
    # (px, py, pz) = (2.12523135442, 1.0323831248, 1.0805572811)
    # (roll, pitch, yaw) = (0.0261365985352, -0.11841110929, 0.0785636422627)
    #
    # (px, py, pz) = (2.1132217851, 0.967987193221, 0.942915752317)
    # (roll, pitch, yaw) = (0.0140962538429, -0.0582047703001, 0.0399534311959)
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
    R0_G = Rxyz.evalf(subs={R: roll, P: pitch, Y: yaw})

    # Calculate Rrpy
    R_corr = R_z * R_y
    R_corr = R_corr.evalf(subs={P: -(pi/2), Y: pi})
    Rrpy = R0_G[0:3, 0:3] * R_corr

    wx = px - (.303) * R0_G[0, 0]
    wy = py - (.303) * R0_G[1, 0]
    wz = pz - (.303) * R0_G[2, 0]

    theta1 = atan2(wy, wx)

    # Find the origin of the second link using theta1
    t0_2 = T0_2.evalf(subs={q1: theta1})
    o2 = [t0_2[0, 3], t0_2[1, 3], t0_2[2, 3]]

    l2 = sqrt((o2[0] - wx)**2 + (o2[1] - wy)**2 + (o2[2] - wz)**2)
    l3 = sqrt(s[a3]**2 + s[d4]**2)

    D1 = (l2**2 - s[a2]**2 - l3**2) / (2 * s[a2] * l3)  # http://smpp.northwestern.edu/savedLiterature/Spong_Textbook.pdf

    alpha = atan2(sqrt(1 - D1**2), D1)
    beta = atan2(s[a3], s[d4])

    theta3 = (alpha + beta) - (pi / 2)

    D2 = (s[a2]**2 + l2**2 - l3**2) / (2 * s[a2] * l2)

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

    # theta4, theta5, theta6 = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), axes='rzxy')

    theta1 = np.clip(np.float64(theta1), -3.23, 3.23)
    theta2 = np.clip(np.float64(theta2), -0.79, 1.48)
    theta3 = np.clip(np.float64(theta3), -3.67, 1.13)
    theta4 = np.clip(np.float64(theta4), -6.11, 6.11)
    theta5 = np.clip(theta5, -2.18, 2.18)
    theta6 = np.clip(np.float64(theta6), -6.11, 6.11)

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
    parser = OptionParser()
    parser.add_option("-d", action="store_true", dest="run_debug", default=False)
    (options, args) = parser.parse_args()
    if options.run_debug:
        print 'In debug mode'
        debug_calculate_IK()
    else:
        print 'Starting IK server'
        IK_server()