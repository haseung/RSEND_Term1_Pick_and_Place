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
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Author for Forward / Inverse Kinematics: Harrison Seung
        ### Forward Kinematics
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    	# Create Modified DH parameters with initialized values for thetas/qi
        s = {alpha0:        0,  a0:        0,   d1:  0.75,  q1: 0,
             alpha1:    -pi/2,  a1:     0.35,   d2:     0,  q2: q2-pi/2,
             alpha2:        0,  a2:     1.25,   d3:     0,  q3: 0,
             alpha3:    -pi/2,  a3:   -0.054,   d4:  1.50,  q4: 0,
             alpha4:     pi/2,  a4:        0,   d5:     0,  q5: 0,
             alpha5:    -pi/2,  a5:        0,   d6:     0,  q6: 0,
             alpha6:        0,  a6:        0,   d7: 0.303,  q7: 0}

	# Define Modified DH Transformation matrix
    	def TF_Matrix(alpha, a, d, q):
            TF = Matrix([[			  cos(q),           -sin(q),           0,             a],
                         [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                       	 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                       	 [                 0,                 0,           0,             1]])
            return TF

	# Create individual transformation matrices
    	T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(s)
        	
	#Generalized transformation matrix from base link to end effector
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE 

        # Extract rotation matrices from the transformation matrices
        RO_EE = T0_EE[0:3, 0:3]

        ### End Forward Kinematics

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

        ### Inverse Kinematics
        #Compensate for rotation discrepancy between DH parameters and Gazebo
        R_z = Matrix([[     cos(np.pi), -sin(np.pi),             0,  0],
                      [     sin(np.pi),  cos(np.pi),             0,  0],
                      [              0,           0,             1,  0],
                      [              0,           0,             0,  1]])
        R_y = Matrix([[  cos(-np.pi/2),           0, sin(-np.pi/2),  0],
                      [              0,           1,             0,  0],
                      [ -sin(-np.pi/2),           0, cos(-np.pi/2),  0],
                      [              0,           0,             0,  1]])
        R_corr = simplify(R_z * R_y)

	# Calculate joint angles using Geometric IK method
        #SSS triangle sides created between Joint 2, Joint 3, and the WC.  See README for figures and hand calculations.
        side_a = sqrt(pow((WC[2]-0.75), 2) + pow((WC[0]-0.35), 2))  # length from Joint 2 to WC
        side_b = 1.501 # equivalent to sqrt(a3*a3 + d4*d4)
        side_c = 1.25 # equivalent to a2

        #SSS triangle angles created between Joint 2, Joint3, and the WC.  See README for figures and hand calculations.
        angle_A = acos((pow(side_b, 2) + pow(side_c, 2) - pow(side_a, 2)) / (2 * side_b * side_c)) #angle between side_b and side_c
        angle_B = acos((pow(side_a, 2) + pow(side_c, 2) - pow(side_b, 2)) / (2 * side_a * side_c)) #angle between side_a and side_c
        angle_C = acos((pow(side_a, 2) + pow(side_b, 2) - pow(side_c, 2)) / (2 * side_a * side_b)) #angle between side_a and side_b
        gamma1 = atan2(0.054, 1.50) #Offset angle calculated by atan2(a3, d4) for determining theta3
        gamma2 = atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC [1]) - 0.35) #Offset angle for determining theta2

	#theta locations shown in README.
        theta1 = atan2(WC[1], WC[0]) 
        theta2 = pi / 2 - (angle_B + gamma2)
        theta3 = (angle_A + gamma1) - pi / 2

	#Calculate rotation matrix for orientation of WC by inverse matrix multiplication of ROT
        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3] #Generalized rotation matrix to WC
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}) #Rotation matrix to WC at current WC position
        R3_6 = R0_3.inv("LU") * ROT_EE #Rotation matrix for orientation of WC

        #Euler angles from rotation matrix
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta5 = atan2(sqrt(pow(R3_6[0,2], 2) + pow(R3_6[2,2], 2)), R3_6[1,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])

	### END Inverse Kinematics
	
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
