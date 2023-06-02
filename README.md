# Assignment 2: Two-Wheeled Mobile Robot
This is a simple, two-wheeled mobile robot that is coded and developed by **Natnael Berhanu Takele**. Additionally, this file contains its software documentation. 

## Assignment Description
The two-wheeled mobile robot that will be used in this assignment moves in three dimensions by avoiding obstacles in order to reach the target position. A 3D simulation environment called *Gazebo* allows the user to operate the robot.

The implementation of an action server that moves a two-wheeled mobile robot in the given environment uses the concept of the *bug0 algorithm*.

Bug0 Algorithm:
- head toward goal
- follow obstacles until you can head toward the goal again
- continue

The created package should contain three different nodes:
1. Node a: A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom.
2. Nobe b: A service node that, when called, prints the number of goals reached and cancelled.
3. Node c: A node that subscribes to the robot’s position and velocity (using the custom message) and prints the distance of the robot from the target and the robot’s average speed.

Additionally, create a launch file to start the whole simulation.

## The Created Nodes
*Nodes* are processes that perform some computation or task. It is a simple program that publishes or subscribes to a topic or contains a program that enables a ROS service. It should be highly specialized and accomplish a single task. Nodes communicate with other nodes by sending messages. As it is clearly seen, we have six Python files in our package. It is basically our node. 

1. **bug_as.py** - in order to move the robot to the designated place, the action server node known as *bug_as.py* must first receive the intended position from the client.
2. **go_to_point_service.py** - this ROS node in Python implements a navigation algorithm that directs a robot to the designated location. Moreover, it is a python script file that implements a service node.
3. **wall_follow_service.py** -  A service node is implemented in this Python script file. This node can be invoked to enable the robot to maneuver around obstacles. The robot navigates by determining the distance to walls to its left, right, and front using laser scan data.
4. **node_a_action_client.py** - It is another python script file that deals with asking the user to enter the coordinates (x, y) or cancel them that the robot has to reach. It creates a publisher "pub" that publishes a custom message *"Posxy_velxy"* on the topic *"/posxy_velxy"*. The custom message contains four fields *"msg_pos_x"*, *"msg_pos_y"*, *"msg_vel_x"*, *"msg_vel_y"* that represent the position and velocity of the robot.

### Pseudocodes
The **pseudocode** for this node is provided over here for an overview of the program.

    Initialize the node
    Create publisher and subscriber

    Define publisher callback function:
        Extract position and velocity data from odometry message
        Create custom message with the data
        Publish the custom message

    Define action client function:
        Create action client and connect to action server
        Loop:
            Wait for user input
            If input is "c":
                Cancel goal
            Else:
                Convert input to goal message
                Send goal to the action server

    Call action client function

5. **node_b_print_goal_r_c.py** - It is a service node, when it is called, it prints the number of goal reached and cancelled. The script creates a service that listens to the */reaching_goal/result* topic and counts the number of goals that have been cancelled and reached.
6. **node_c_print_dist_spd.py** - The robot's average speed and distance from the target are printed out by using this node. These parameters are taken from the */posxy_velxy* topic as a custom message.


## Launch File

A ROS launch file will allow you to start everything you need, from just one file. You can create as many parameters and start as many nodes as you want. You can also create groups of parameters and nodes with a prefix. You can even use some conditional statements, and combine multiple launch files inside one launch file. This way, you will be able to quickly launch your application, and to create different launch files for different startup modes of your robot. *roslaunch* is a tool for easily launching multiple ROS nodes locally, as well as setting parameters on the Parameter Server. It includes options to automatically respawn processes that have already died.

Our ROS launch file is:    
>assignment2.launch


           <?xml version="1.0"?>
           <launch>
                <include file="$(find assignment_2_2022)/launch/sim_w1.launch" />
                <param name="desired_pos_x" value= "0.0" />
                <param name="desired_pos_y" value= "1.0" />
                <param name="frequency" type="double" value="1.0" />
    
                <node pkg="assignment_2_2022" type="wall_follow_service.py" name="wall_follower" />
                <node pkg="assignment_2_2022" type="go_to_point_service.py" name="go_to_point"  />
                <node pkg="assignment_2_2022" type="bug_as.py" name="bug_action_service" output="screen" />
                <node pkg="assignment_2_2022" type="node_a_action_client.py" name="node_a_action_client" output="screen" launch-prefix="xterm -hold -e" />
                <node pkg="assignment_2_2022" type="node_b_print_goal_r_c.py" name="node_b_print_goal_r_c"  />
                <node pkg="assignment_2_2022" type="node_c_print_dist_spd.py" name="node_c_print_dist_spd" output="screen" launch-prefix="xterm -hold -e" />
           </launch>


## Installing and Running

Before doing anything, run the master by using this command:
>roscore &

It is vital to install the xterm library by using the following command to print out the outputs for each node.
>sudo apt-get install xterm -y

Then, use the following commands to run it smoothly:
1. Duplicate your repository in the *src* folder of your workspace
>git clone https://github.com/NatnaelB7/Assignment_2_TAKELE_RT1.git
2. Build the workspace in your root folder
>catkin_make
3. Launch the simulation
>rospack find assignment_2_2022

>source /root/Assignment_2/devel/setup.bash

>roslaunch assignment_2_2002 assignment2.launch

## Possible Improvements

I suggest some possible improvements for this assignment. These are:

- To clearly see the robot's goal, we may place a graphic marker in the environment that depicts the position it needs to achieve.
- Integrating the robot's position in the display, whether by pointing an arrow in that direction or showing a 3D model of the robot.
- The robot usually decides for itself which path to follow when it comes across a hurdle. It would be better for it to decide which way to go depending on the shortest path to the intended location.

## Conclusion

To conclude, we have created a package that contains three different nodes. Each node performs a specific task. The first node that we developed implements an action client, allowing the user to set a target or cancel it. By relying on the values, it also publishes the robot's position and velocity as a custom message on the topic. The other node prints the number of goals reached and cancelled. The third node prints the robot's average speed and distance from the target after subscribing to the robot's position and velocity (using the custom message). We also created a launch file to start the entire simulation environment and our six different nodes.
