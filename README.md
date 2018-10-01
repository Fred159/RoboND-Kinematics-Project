# Udaicty - Kinematics-Project.

## This project use ROS to give manipulator each joints' target angle, to reach pre-calculated position. 
### keyword: ROS, GAZEBO, RVIZ, Forward kinematic, inverse kinematic, geometric trigonometric function, rotation matrix, translation matrix, DH table

## In order to do this, I followed orders below. 

(Wath all the resource provided by Udacity classroom first)
1. Based on classroome contents, calculate DH table.
2. Use DH table calculate the forward kinematic. Forward kinematic start from base line ( origin coordinate 0) and end with end-effector. FK is a kind of function.
- FK = f(theta1,theta2, ....theta6)

![DH table](https://github.com/Fred159/RoboND-Kinematics-Project/blob/master/my%20screen%20capture/DH%20table.png)

So if 'theta1,theta2, ....theta6'is given , then we can calculate the final position and orientation of end effector.
3. In order to move a target into a specify position, we receive target postion in gazebo simulator. And then, use this postion, we should derive a function to calculate each joint's angle (parameter need to calculate: theta1,theta2, ....theta6). So we need to calculate Inverse kinematic(IK). IK is a function too. 
- theta1,theta2, ....theta6 = IK(target position , target orientation)
4. Use geometric method to derive a function to calculate theta1, theta2, theta3.
Use inverse roration matrix to calculate theta4,theta5,theta6
5. Run roscore and safe_spawner.
![Gazebo simulator](https://github.com/Fred159/RoboND-Kinematics-Project/blob/master/my%20screen%20capture/2018-09-29-053522_1920x984_scrot.png)
6. Run IK_debug.py 
![IK_debug.py code](https://github.com/Fred159/RoboND-Kinematics-Project/blob/master/my%20screen%20capture/2018-09-29-053350_1920x960_scrot.png)
![IK_debug.py result](https://github.com/Fred159/RoboND-Kinematics-Project/blob/master/my%20screen%20capture/2018-09-29-053323_659x408_scrot.png)
7. Run IK_inverse.py


－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－

The parameters is obtained using the following convention


![alt text](https://raw.githubusercontent.com/lisaljl/Udacity-RoboND-Kinematics/master/code/DH.png "DH annotation")
DH table angles in radian.
![DH table from udacity](https://github.com/Fred159/RoboND-Kinematics-Project/blob/master/my%20screen%20capture/DH%20table2.png)

| i        | a(i-1)  | a(i-1)  | d(i)    | θ(i)    |
| :------- |:-------:|:-------:|:-------:|:-------:|
| 1        | 0       | 0       | 0.75    |θ1       |
| 2        | -90     | 0.35    | 0       |θ2 - 90  |
| 3        | 0       | 1.25    | 0       |θ3       |
| 4        | -90     | -0.054  | 1.5     |θ4       |
| 5        | 90      | 0       | 0       |θ5       |
| 6        | -90     | 0       | 0       |θ6       |
| E        | 0       | 0       | 0.303   |0        |


### Forward Kinematics

#### Transformation matrix for Kuka arm
Transfrom T0_1 represent that coordinate transfrom from 0 to 1.
Transfrom Ti-1_i represent that coordinate transfrom from i-1 to i.

```
            T0_1 =  Matrix([
            [cos(q1), -sin(q1), 0,  0],
            [sin(q1),  cos(q1), 0,  0],
            [      0,        0, 1, d1],
            [      0,        0, 0,  1]])

            T1_2 =  Matrix([
            [sin(q2),  cos(q2), 0, a1],
            [      0,        0, 1,  0],
            [cos(q2), -sin(q2), 0,  0],
            [      0,        0, 0,  1]])
	    
            T2_3 =  Matrix([
            [cos(q3), -sin(q3), 0, a2],
            [sin(q3),  cos(q3), 0,  0],
            [      0,        0, 1,  0],
            [      0,        0, 0,  1]])	    

            T3_4 =  Matrix([
            [ cos(q4), -sin(q4), 0, a3],
            [       0,        0, 1, d4],
            [-sin(q4), -cos(q4), 0,  0],
            [       0,        0, 0,  1]])

            T4_5 =  Matrix([
            [cos(q5), -sin(q5),  0, 0],
            [      0,        0, -1, 0],
            [sin(q5),  cos(q5),  0, 0],
            [      0,        0,  0, 1]])

            T5_6 =  Matrix([
            [ cos(q6), -sin(q6), 0, 0],
            [       0,        0, 1, 0],
            [-sin(q6), -cos(q6), 0, 0],
            [       0,        0, 0, 1]])

            T6_G =  Matrix([
            [0,  0, 1,  0],
            [0, -1, 0,  0],
            [1,  0, 0, dE],
            [0,  0, 0,  1]])

            
```
#### Transformation matrix for base_link to all links

```
     T0_2 = T0_1*T1_2 #base_link to link_2
     T0_3 = T0_2*T2_3 #link_2 to link_3
     T0_4 = T0_3*T3_4 #link_3 to link_4
     T0_5 = T0_4*T4_5 #link_4 to link_5
     T0_6 = T0_5*T5_6 #link_5 to link_6
     T0_G = simplify(T0_6*T6_G) #link_6 to link_G
```

#### Homogeneous transform matrix from base_link to gripper_link
In this case, alpha has been replaced with the following
```
s = {
     alpha0: 0, 
     alpha1: -pi/2,  
     alpha2: 0, 
     alpha3: -pi/2, 
     alpha4: pi/2, 
     alpha5: -pi/2}
```

### Inverse Kinematics
Ik is obtained by splitting into two parts, this is to reduce the complexity of the calculations. One part is a RRR revolute arm, another a spherical arm. Solving the RRR revolute arm would return the joint angles for joint 1,2 and 3. While the spherical arm would give joint angles for joint 4,5 and 6

#### RRR joint

![alt text](https://raw.githubusercontent.com/lisaljl/Udacity-RoboND-Kinematics/master/code/ik_q2q3.png "q2q3")
We would ignore joint 4,5,6. Effectively imagine an arm with just joint 1,2,3

##### Angle for joint 1
This is the easiest angle to determine as it just atan2(y, x)

##### Angle for joint 2 and 3
We discard joint 1 for now. To calculate the angles for these two joints, it is about applying the cosine rule to obtain the angle first for angle 3, then calculating angle 2. The figure below explains how theta is obtained

![alt text](https://raw.githubusercontent.com/lisaljl/Udacity-RoboND-Kinematics/master/code/ik_q2q3_triangle.png "q2q3")

Forming this equation
```
l2 = a2
l3 = sqrt(a3^2 + d4^2)
β = 180 - 𝚹3
xy = sqrt(Wc_x^2 + Wc_y^2) - a0
z = Wc_z - d1
```
Using the cosine rule
```
D^2 = xy^2 + z^2 = l2^2 + l3 ^2 - 2(l2)(l3)cos(β)
cos(𝚹3) = (xy^2 + z^2 - l3*l3 - l2*l2)/(2*l3*l2) = r
𝚹3 = atan2(sqrt(1-r*r), r), where sqrt(1-r*r) = sin(𝚹3)
𝚹3 = atan2(-sqrt(1-r*r), r)
𝚹2 = γ - ⍺ = atan2(xy, z) - atan2(l3*sin(𝚹3), l2+l3*cos(𝚹3))
𝚹3 = 𝚹3 - pi/2
```
𝚹3 has to be be minus with 90 degrees, as the start point is not vertical up, but horizontal right

𝚹3 is checked to ensure it is between -pi and pi/2, as it starts horizontally
𝚹2 is checked to be -pi to pi

##### Angle for joint 4,5,6
The overall roll, pitch, yaw for the end effector relative to the base link is as follows:
![alt text](https://raw.githubusercontent.com/lisaljl/Udacity-RoboND-Kinematics/master/code/rot_spherical.png "rotation")

Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:
```
    R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
 ```  
Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:

    R0_6 * Rcorr = Rrpy
    where Rrpy = Homogeneous RPY rotation between base_link and gripper_link
    ajd Rcorr is the difference in axis between the DH convention frame to the URDF frame

As we calculated joints 1-3, we can substitute those values in their respective individual rotation matrices and pre-multiply both sides of eq1 by inv(R0_3) which leads to:
```
    R0_3 * R3_6 * Rcorr = Rrpy
    R3_6 = inv(R0_3) * Rrpy * Rcorr.T
    Rcorr = rot_z()*rot_y()
 ```   
Note for Rrpy we are using extrinsic rotation for X-Y-Z, as by default the method tf.transformation_matrix("","rxyz"), by default returns roll, pitch, yaw for an extrinsic rotation of  X-Y-Z. As such the inverse rotation matrix is 

```
Rxyz_ext = Rx(roll) * Ry(pitch) * Rz(yaw)
inv(Rxyz_ext) = Rz(yaw) * Ry(pitch) * Rx (roll)

    Rrpy = Matrix([[cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma)], 
               [sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma)],
               [          -sin(beta),                                    cos(beta)*sin(gamma), cos(beta)*cos(gamma)]]);

𝚹4 = atan(sin(q4)*sin(q5), -sin(q5)*cos(q4))
where both sin(q5) cancels out

𝚹6 = atan(-sin(q5)*sin(q6), sin(q5)*cos(q6))
where both sin(q5) cancels out

𝚹5 = atan(sqrt(sin2(q5)*cos2(q6) + sin2(q5)*sin2(q6)), cos(q5)) = atan(sin(q5), cos(q5))
where sin2 + cos2 = 1

```
－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－



## Simulation results.
Simulation result shows a good grip performance.
![Simulation screencapture](https://github.com/Fred159/RoboND-Kinematics-Project/blob/master/my%20screen%20capture/2018-09-29-053554_1920x984_scrot.png)
![Rviz run capture](https://github.com/Fred159/RoboND-Kinematics-Project/blob/master/my%20screen%20capture/2018-09-29-053611_1920x984_scrot.png)


[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.

