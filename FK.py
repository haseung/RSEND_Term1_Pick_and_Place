import numpy as np
from time import time
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

start_time = time()

### Create symbols for joint variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

### Kuka KR210 ###
#DH Parameters, initiallize q to zero
s = {alpha0:        0,  a0:        0,   d1:  0.75,  q1: 0,
     alpha1:    -pi/2,  a1:     0.35,   d2:     0,  q2: q2-pi/2,
     alpha2:        0,  a2:     1.25,   d3:     0,  q3: 0,
     alpha3:    -pi/2,  a3:   -0.054,   d4:  1.50,  q4: 0,
     alpha4:     pi/2,  a4:        0,   d5:     0,  q5: 0,
     alpha5:    -pi/2,  a5:        0,   d6:     0,  q6: 0,
     alpha6:        0,  a6:        0,   d7: 0.303,  q7: 0}

#### Homogeneous Transforms
# base_link to link1
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[			  cos(q),           -sin(q),           0,             a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
               	 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
               	 [                 0,                 0,           0,             1]])
    return TF

T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(s)

# Composition of Homogeneous Transform
T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
T0_3 = simplify(T0_2 * T2_3) # base_link to link_3
T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
T0_EE = simplify(T0_6 * T6_EE) # base_link to end effector

# Correction Needed to Account of Orientation Difference Between Definition of
# End Effector Link in URDF versus DH convention
R_z = Matrix([[     cos(np.pi), -sin(np.pi),             0,  0],
              [     sin(np.pi),  cos(np.pi),             0,  0],
              [              0,           0,             1,  0],
              [              0,           0,             0,  1]])
R_y = Matrix([[  cos(-np.pi/2),           0, sin(-np.pi/2),  0],
              [              0,           1,             0,  0],
              [ -sin(-np.pi/2),           0, cos(-np.pi/2),  0],
              [              0,           0,             0,  1]])
R_corr = simplify(R_z * R_y)

# Total Homogeneous Transform Between base_link and End_Effector_link with
# orientation correction applied
T_total = simplify(T0_EE * R_corr)

# Calculate Wrist Center (WC) location
EE_pos = T_total[0:3,3]
R0_6 = T0_6[0:3, 0:3]
dG = Matrix([[0], [0], [0.303]])
WC = EE_pos - R0_6.evalf(subs={q2: 0}) * dG

#### Numerically evaluate transforms (compare with output of tf_echo)
#print("T0_1 = ", T0_1.evalf(subs={q2: 0}))
#print("T0_2 = ", T0_2.evalf(subs={q2: 0}))
#print("T0_3 = ", T0_3.evalf(subs={q2: 0}))
#print("T0_4 = ", T0_4.evalf(subs={q2: 0}))
#print("T0_5 = ", T0_5.evalf(subs={q2: 0}))
#print("T0_6 = ", T0_6.evalf(subs={q2: 0}))
#print("T0_EE = ", T0_EE.evalf(subs={q2: 0}))
print("T_total = ", T_total.evalf(subs={q2: 0}))
print ("End Effector position is", EE_pos.evalf(subs={q2: 0}))
print ("Wrist center position is", WC.evalf(subs={q2: 0}))
#print ("Total run time is %04.4f seconds\n" % (time()-start_time))
