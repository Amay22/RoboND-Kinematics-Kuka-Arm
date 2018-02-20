## Project: Kinematics Pick & Place

---

**Steps to complete the project:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 

[//]: # (Image References)

[image1]: ./misc_images/robot_arm2.png
[image2]: ./misc_images/dh.png
[image3]: ./misc_images/DH-vars.png
[image4]: ./misc_images/joint_variables.png
[image5]: ./misc_images/joint_vars.png
[image6]: ./misc_images/dh-transform-matrix.png
[image7]: ./misc_images/DH-workout-1.jpg
[image8]: ./misc_images/DH-workout-2.jpg
[image9]: ./misc_images/joint_angles.png
[image10]: ./misc_images/joint_diagram.jpg
[image11]: ./misc_images/flow_diagram.png
[image12]: ./misc_images/IK-workout-1.jpg
[image13]: ./misc_images/IK-workout-2.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

This goal of this project is to control the end point effector of a robotic arm that is used to pick and place objects using the robot operating system (ROS). This project uses the Kuka KR210, a six degree of freedom (6DOF) revolute joint or an RRRRRR robot. In this project we'll using the details about this robotic arm i.e. the
 angles of each joint, totalling 6 joints (i.e. 6 Degrees of Freedom).

Forward Kinematics (FK) is a method to calculate the final coordinate position and rotation of end-effector of a bunch of conjoined links (e.g. robotic arms, limbs, etc.) given the parameters of each joint between links. Using Forward kinematics we can calculate the grippers location if we know the joint angles θ. 

Inverse Kinematics (IK), is the exact opposite of FK, where we calculate the parameters from a given coordinate position and rotation. We can calculate the the joint angles if we know the gripper location. 

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![alt text][image2]

To do FK and IK, we are using a method by Jacques Denavit and Richard Hartenberg which requires the following parameters:

![alt text][image3]

![alt text][image4]

alpha : angle about common normal, from old z axis to new z axis
a: length of the common normal
d : offset along previous z to the common normal
theta : angle about previous z, from old  x to new x

Modified Denavit-Hartenberg parameters

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | θ1
1->2 | -pi/2 | 0.35 | 0 | θ2 - pi/2
2->3 | 0 | 1.25 | 0 | θ3
3->4 |  -pi/2 | -0.054 | 1.5 | θ4
4->5 | pi/2 | 0 | 0 | θ5
5->6 | -pi/2 | 0 | 0 | θ6
6->EE | 0 | 0 | 0.303 | 0

![alt text][image5]

```python
# Create Modified DH parameters
DH = {   alpha0: 0,      a0: 0,      d1: 0.75,    q1: q1,
         alpha1: -pi/2,  a1: 0.35,   d2: 0,       q2: q2-pi/2,
         alpha2: 0,      a2: 1.25,   d3: 0,       q3: q3,
         alpha3: -pi/2,  a3: -0.054, d4: 1.5,     q4: q4,
         alpha4: pi/2,   a4: 0,      d5: 0,       q5: q5,
         alpha5: -pi/2,  a5: 0,      d6: 0,       q6: q6,
         alpha6: 0,      a6: 0,      d7: 0.303,   q7: 0}
```

![alt text][image7]
![alt text][image8]


The values for a and d are in this file : [kr210.urdf.xacro](https://github/amay22/) .


After setting the DH parameters we can apply it on the Homogenous transforms. Each transformation matrix looks like this:

![alt text][image6]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Forward kinematics can be used to calculate the gripper coordinates in the 3D plane when we know the joint angles θ. However, with a pick and place robot arm, we only know the position of the object we require to pick up. Hence to get the gripper to go towards the object we need to pick up we can can calculate the angles required to have the gripper in the desired position. This is where inverse kinematics comes into play. We rely on trigonometry heavily to get all these joint angles θ.

![alt text][image9]
![alt text][image10]
![alt text][image11]

We then need to calculate the joint angles θ i.e. theta. The first 3 theta angle's comprise of the base link. The inverse joint angle the this base section can then be derived by using basic trigonometry and the law of cosine. The remaining 3 theta joints and can be derived using trigonometric laws and current robot orientation and Position.

θ1: angle from position in the x-y plane
θ2: law of cosine in the x, y, z planes and perpendicular to the z-plane.
θ3: the difference in the law of cosine angle a continuation of link 2.
θ4: it'll be on a tangent to the center of the pickup location i.e. at a zero angle when at the bottom, pi/2 rotation angle on the left side, pi rotation when verticle and 3*pi/2 on the right side.
θ5: derived from the most direct line to the target object.
θ6: negative of theta 4's angle.

My handwriting is awful even I don't understand it myself but the code is readable.

![alt text][image12]
![alt text][image13]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

`IK_sever.py` starts of by processing the request parameter that is passed to it. . The request contains an arm coordinates i.e. the x, y, & z positions. We use the proposed IK mentioned above to get the valid joint angles for the 6 links. The first joint angles are the arm angles and we can process them using the trigonometry working that I have hand-written above. 

θ1 angle is relative to the base frame and it's calculation is straigtforward. For θ2 and θ3 we calculate the offset  that corresponds to the rotations of links w.r.t. x axis. After that we use the cosine rule upon the the link 3 and the length of the arms itself to calculate the θ2 and θ3. All this can be found in `calculate_arm_IK()` function.

Now I calculated the remaining joint angles that cmoprise of the write of the arm. The wrist rotation angle θ4 is calculated by creating a reference plane that represents the target-frame (object to pickup) space. θ4 is calculated so to keep θ5's z-axis perpendicular to the target-frame. θ5 is the arm end and it is kept within a range to keep the gripper level constant while moving. θ6 is inversse of θ4 to keep the gripper claws level. All this can be found in `calclate_wrist_speherical_IK()` function.

For debugging I have also FK code. Once we get all the joint angles i.e. θ (1-6) we can substitute these IK values in modified DH FK equation also described above.  This is illustrated in `calculate_DH_FK()`.