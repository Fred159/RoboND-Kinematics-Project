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

