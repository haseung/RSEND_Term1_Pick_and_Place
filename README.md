## Project: Kinematics Pick & Place
### Writeup / README
### Author: Harrison Seung
### Date: 07/18/2018

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

The following README is for the Udacity Robotics Software Engineering Nanodegree, Term 1, Project 2, Pick and Place.  

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

A kinematic analysis was performed on the Kuka kr210  6 degree of freedom robot manipulator using the modified Denavit-Hartenberg parameters.  The figure below indicates locations for each joint origin and corresponding axes.  The DH parameter table is show below per the convention used in the Udacity notes (Craig)

![DH Parameter Notation Locations](https://github.com/haseung/term1_project2_pickandplace/blob/master/DH%20Figure.JPG)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between the base_link and gripper_link using only the end-effector(gripper) pose.

Using the DH parameter table, individual transformation matrices for each joint were created for the starting position:

base_link to link1:

|     |   l |    m|    n|    P|
| ----| ----| ----| ----| ----|
|    x|  1.0|    0|    0|    0| 
|    y|    0|  1.0|    0|    0| 
|    z|    0|    0|  1.0| 0.75| 
|     |    0|    0|    0|    0| 

base_link to link2:

|     |   l |    m|    n|    P|
| ----| ----| ----| ----| ----|
|    x|    0|  1.0|    0| 0.35| 
|    y|    0|    0|  1.0|    0| 
|    z|  1.0|    0|    0| 0.75| 
|     |    0|    0|    0|  1.0| 

base_link to link3:

|     |   l |    m|    n|    P|
| ----| ----| ----| ----| ----|
|    x|    0|  1.0|    0| 0.35| 
|    y|    0|    0|  1.0|    0| 
|    z|  1.0|    0|    0|    2| 
|     |    0|    0|    0|  1.0| 

base_link to link4:

|     |   l |    m|    n|     P|
| ----| ----| ----| ----|  ----|
|    x|    0|    0|  1.0|  1.85| 
|    y|    0| -1.0|    0|     0| 
|    z|  1.0|    0|    0| 1.946| 
|     |    0|    0|    0|   1.0| 

base_link to link5:

|     |   l |    m|    n|     P|
| ----| ----| ----| ----|  ----|
|    x|    0|  1.0|    0|  1.85| 
|    y|    0|    0|  1.0|     0| 
|    z|  1.0|    0|    0| 1.946| 
|     |    0|    0|    0|   1.0|

base_link to link6:

|     |   l |    m|    n|     P|
| ----| ----| ----| ----|  ----|
|    x|    0|    0|  1.0|  1.85| 
|    y|    0| -1.0|    0|     0| 
|    z|  1.0|    0|    0| 1.946| 
|     |    0|    0|    0|   1.0|

base_link to gripper_link/end effector:

|     |   l |    m|    n|     P|
| ----| ----| ----| ----|  ----|
|    x|    0|    0|  1.0| 2.153| 
|    y|    0| -1.0|    0|     0| 
|    z|  1.0|    0|    0| 1.946| 
|     |    0|    0|    0|   1.0|

The generalized homogeneous transform between the base_link and gripper_link is:

|     |   l |    m|    n|     P|
| ----| ----| ----| ----|  ----|
|    x|  1.0|    0|    0| 2.153| 
|    y|    0|  1.0|    0|     0| 
|    z|    0|    0|  1.0| 1.946| 
|     |    0|    0|    0|   1.0|

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The inverse kinematics of the KR210 robot for theta1, theta2, and theta3 were determined using the geometric method shown below:

![Geometric Method](https://github.com/haseung/term1_project2_pickandplace/blob/master/geometricmethod.jpg)

Theta4, theta5, and theta6 are determined by using the Euler Angles for a Rzyx extrinsic rotation.  The method involves finding R3_6 by pre-multipying the R0_6 rotation matrix by the inverse/transpose of R0_3.  Then theta 1, 2, 3 are substituted into R3_6 resulting in an exact solution.  Additionally R3_6 can be found generally using the Euler Angle for a Rzyx extrinsic rotation.  Solving for theta 4, 5, and 6 as shown in the figure below and equating each method for R3_6 allows us to solve for the exact solutions of theta 4, 5, and 6.   

![EulerAngles](https://github.com/haseung/term1_project2_pickandplace/blob/master/EulerAngles.jpg)

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

The IK_server.py file implements the forward and inverse kinematics analysis for the KR210 robot simulation to provide required joint angles for planned trajectory for object retrieval and delivery.  When a valid position is received the handle_calculate_IK function begins at Line 32 by first calculating the Forward Kinematics.  This is done by:
1. Establishing the DH Table parameters
2. Defining the TF_Matrix function for use in calculating each individual transformation Matrix
3. Calculating each individual transformation Matrix
4. Combining the transformation matrix to form a generalized transformation matrix from the base_link to the End_Effector_link
5. Extracting necessary rotation matrices from the final transforms

Next the inverse kinematics are calculated.  This is done by:
1. The rotation matrix is calculated to compensate for the DH parameter vs Gazebo orientation Offset.
2. The first 3 joint angles are determined using the Geometric IK method.  
3. The last 3 joint angles are determined using the Euler Angles from Rotation Matrix approach.    

Finally, once all 6 joint angles are returned for use in the joint_trajector_list.

![PickPlaceStep1](https://github.com/haseung/term1_project2_pickandplace/blob/master/PickPlaceStep1.JPG)
KR210 Retrieving randomly placed object

![PickPlaceStep2](https://github.com/haseung/term1_project2_pickandplace/blob/master/PickPlaceStep2.JPG)
KR210 Returning placed object to dropoff location

![PickPlaceStep3](https://github.com/haseung/term1_project2_pickandplace/blob/master/PickPlaceStep3.JPG)
KR210 returning to start position after successful delivery of object

![CompletedPickPlace](https://github.com/haseung/term1_project2_pickandplace/blob/master/CompletedPickPlace.JPG)
KR210 successfully retrieving 9/10 objects
