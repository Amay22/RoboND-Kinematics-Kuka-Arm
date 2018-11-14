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
[image14]: ./misc_images/R36.png
[image15]: ./misc_images/R3_6.png
[image16]: ./misc_images/pick1.png
[image17]: ./misc_images/pick2.png
[image18]: ./misc_images/place1.jpg


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


To do FK and IK, we are using a method by Jacques Denavit and Richard Hartenberg which requires the following parameters:

![alt text][image2]

![alt text][image3]

![alt text][image4]

![alt text][image5]

The values for a and d are in this file : [kr210.urdf.xacro](https://github.com/Amay22/RoboND-Kinematics-Kuka-Arm/blob/master/kuka_arm/urdf/kr210.urdf.xacro). With the help of these parameters, and a the diagram above showing all the joints in the unactuated pose (all joint angles equal to zero, the DH parameter table was obtained.

`alpha` : angle about common normal, from old z axis to new z axis

`a`: length of the common normal

`d` : offset along previous z to the common normal

`theta` : angle about previous z, from old  x to new x

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


From the [kr210.urdf.xacro](https://github.com/Amay22/RoboND-Kinematics-Kuka-Arm/blob/master/kuka_arm/urdf/kr210.urdf.xacro) we get:

```html
  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
  </joint>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
  </joint>
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
  </joint>
  <joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
  </joint>
  <joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
  </joint>

```

Hence, the parameters are as follows:


```python
a_1 = 0.35
a_2 = 1.25
a_3 = -0.054
d_1 = 0.75
d_3 = 1.5
d_6 = 0.303
```


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The way DH parameters are defined here, the transformation matrix could be obtained as follows:

1. Rotate about x[i-1] by alpha[i-1]
2. Translate along x[i-1] by a[i-1]
3. Translate along axis z[i] by d[i]
4. Rotate about resulting axis z[i] by theta[i]


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


After setting the DH parameters we can apply it on the Homogenous transforms. Each transformation matrix looks like this:

![alt text][image6]

I have put in the [calculate_DH_FK()](https://github.com/Amay22/RoboND-Kinematics-Kuka-Arm/blob/master/kuka_arm/scripts/IK_server.py#L184-L249) function. Please feel free to uncomment and observe the output.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Forward kinematics can be used to calculate the gripper coordinates in the 3D plane when we know the joint angles θ. However, with a pick and place robot arm, we only know the position of the object we require to pick up. Hence to get the gripper to go towards the object we need to pick up we can can calculate the angles required to have the gripper in the desired position. This is where inverse kinematics comes into play. We rely on trigonometry heavily to get all these joint angles θ.

![alt text][image9]
![alt text][image10]
![alt text][image11]

We then need to calculate the joint angles θ i.e. theta. The first 3 theta angle's comprise of the base link. The inverse joint angle the this base section can then be derived by using basic trigonometry and the law of cosine. The remaining 3 theta joints and can be derived using trigonometric laws and current robot orientation and Position.

`θ1`: angle from position in the x-y plane

`θ2`: law of cosine in the x, y, z planes and perpendicular to the z-plane.

`θ3`: the difference in the law of cosine angle a continuation of link 2.

`θ4`: it'll be on a tangent to the center of the pickup location i.e. at a zero angle when at the bottom, pi/2 rotation angle on the left side, pi rotation when verticle and 3*pi/2 on the right side.

`θ5`: derived from the most direct line to the target object.

`θ6`: negative of theta 4's angle.

My handwriting is awful even I don't understand it myself but the code is readable.

![alt text][image12]



`θ1`, `θ2` and `θ3` are fouund straightfoward in the code using the geometric IK method and the cosine law. It can be seen the code in the function [calculate_arm_IK(px, py, pz)](https://github.com/Amay22/RoboND-Kinematics-Kuka-Arm/blob/master/kuka_arm/scripts/IK_server.py#L96-L141) and also as follows:

```python
# px,py,pz = end-effector position (we know this from the robot)
# theta1 Calculate joint angles using Geometric IK method
theta1 = threshold_angle(atan2(py, px), -185, 185)

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

```
#### Inverse orientation problem

The first three thetas can be easily calculated from the end effector position but the same doesn't work for the remaining three theta angles. `θ4`, `θ5` and `θ6`
needs to be solved using the inverse orientation problem.

We need to calculate `R3_6 = inv(R0_3)*Rrpy`, where we already have the thetas to calculate `R0_3` and `Rrpy` come from the desired roll pitch yaw angles. Once we have these two matrices we can compare them and find the equations to calculate theta4, theta5 and theta6. Where R0_6 is the homogeneous RPY rotation matrix calculated above from the base_link to gripper_link.

![alt text][image14]

![alt text][image15]

![alt text][image13]


`θ4`, `θ5` and `θ6` can be seen the code in the function [calclate_wrist_speherical_IK(px, py, pz, theta1, psi2, beta2, psi3)](https://github.com/Amay22/RoboND-Kinematics-Kuka-Arm/blob/master/kuka_arm/scripts/IK_server.py#L143-L182) and also as follows:

```python
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

```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

[IK_sever.py](https://github.com/Amay22/RoboND-Kinematics-Kuka-Arm/blob/master/kuka_arm/scripts/IK_server.py) starts of by processing the request parameter that is passed to it. . The request contains an arm coordinates i.e. the x, y, & z positions. We use the proposed IK mentioned above to get the valid joint angles for the 6 links. The first joint angles are the arm angles and we can process them using the trigonometry working that I have hand-written above.

θ1 angle is relative to the base frame and it's calculation is straigtforward. For θ2 and θ3 we calculate the offset  that corresponds to the rotations of links w.r.t. x axis. After that we use the cosine rule upon the the link 3 and the length of the arms itself to calculate the θ2 and θ3. All this can be found in `calculate_arm_IK()` function.

Now I calculated the remaining joint angles that cmoprise of the write of the arm. The wrist rotation angle θ4 is calculated by creating a reference plane that represents the target-frame (object to pickup) space. θ4 is calculated so to keep θ5's z-axis perpendicular to the target-frame. θ5 is the arm end and it is kept within a range to keep the gripper level constant while moving. θ6 is inversse of θ4 to keep the gripper claws level. All this can be found in `calclate_wrist_speherical_IK()` function.

For debugging I have also FK code. Once we get all the joint angles i.e. θ (1-6) we can substitute these IK values in modified DH FK equation also described above.  This is illustrated in `calculate_DH_FK()`.


My initial code was working but it was not picking up the objects properly. It was either not grasping the objects correctly and it was toppling them over to the other side. My initial code picked up around 5 to 6 objects out of 11.  That's when I wrote the [calculate_DH_FK(q0, q1, q2, q3, q4, q5)](https://github.com/Amay22/RoboND-Kinematics-Kuka-Arm/blob/master/kuka_arm/scripts/IK_server.py#L184-L249) function, which was extremely helpful in debuggin and calculating the modified DH forward kinematics values and I could compare with my Inverse Kinematics value.


I had to fix the thresholding logic and also some of the angles had gone haywire due to my mistakes in signs i.e. positive and negative. My initial code was calculating all thetas in one function but then I separated them out so that I don't get confused. Once I had refined my code the first three thetas and theta 6 were simple geometric functions and . It was theta 4 and 5 that took the most amount of time.

The kuka robot can sucessfully pick up 9 out of 11 pieces and drop them in the bin. The motion from the pick to the arm adjust itself is extremely long and awry but it does straighten out and finally puts it into the bin.

- In try 1: I got 4 out of 8 pieces in the bin. The issue was it was not able to grasp the piece correctly.
- After fixing the code I got 5 out of 8 pieces in the bin
- I then added the Forward kinematics debugging and realizing my mistake I got 7 out of 10 in the bin.
- I tried again without changing the code I got 9 out of 11 pieces. It's somehow not consistent everytime I run the code.

![alt text][image16]
![alt text][image17]
![alt text][image18]

