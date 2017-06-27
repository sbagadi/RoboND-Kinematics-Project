#!/user/bin/env python

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

# Create symbols for joint variables.
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  ## Theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# KUKA KR210
# DH Parameters
s = {alpha0:      0, a0:      0, d1:  0.75,
     alpha1:  -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
     alpha2:      0, a2:   1.25, d3:     0,
     alpha3:      0, a3: -0.054, d4:   1.5,
     alpha4:   pi/2, a4:      0, d5:     0,
     alpha5:  -pi/2, a5:      0, d6:     0,
     alpha6:      0, a6:      0, d7: 0.303, q7: 0}

# Homogeneous Transforms
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

# Composition of Homogeneous Transforms.
T0_2 = simplify(T0_1 * T1_2)  # base_link to link_2
T0_3 = simplify(T0_2 * T2_3)  # base_link to link_3
T0_4 = simplify(T0_3 * T3_4)  # base_link to link_4
T0_5 = simplify(T0_4 * T4_5)  # base_link to link_5
T0_6 = simplify(T0_5 * T5_6)  # base_link to link_6
T0_G = simplify(T0_6 * T6_G)  # base_link to gripper

# Correction needed to account for the orientation difference between definition of Gripper Link in URDF versus DH
# convention.
R_z = Matrix([[    cos(np.pi), -sin(np.pi),             0,      0],
              [    sin(np.pi),  cos(np.pi),             0,      0],
              [             0,           0,             1,      0],
              [             0,           0,             0,      1]])

R_y = Matrix([[ cos(-np.pi/2),           0, sin(-np.pi/2),      0],
              [             0,           1,             0,      0],
              [-sin(-np.pi/2),           0, cos(-np.pi/2),      0],
              [             0,           0,             0,      1]])
R_corr = simplify(R_z * R_y)

# Numerically evaluate transforms (compare this with output of tf_echo!)
print("T0_1 = ", T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_2 = ", T0_2.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_3 = ", T0_3.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_4 = ", T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_5 = ", T0_5.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_6 = ", T0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_G = ", T0_G.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))

# Total homogeneous transform between base_link and gripper_link with orientation correction applied.
T_total = simplify(T0_G * R_corr)
