"""
.. module:: node_a_action_client
   :platform: Unix
   :synopsis: Python module which implements an action client that lets users set or cancel targets (x, y)
   
.. moduleauthor:: Natnael Berhanu Takele

This node deals with asking the user to enter the coordinates (x, y) or cancel them that the robot has to reach. It creates a publisher *pub* that publishes a custom message *Posxy_velxy* on the topic *posxy_velxy*. 

The custom message contains four fields *msg_pos_x*, *msg_pos_y*, *msg_vel_x*, *msg_vel_y* that represent the position and velocity of the robot.

Subscribes to:
   /odom

Publishes to: 
   /posxy_velxy   
   
Action Client:
   /reaching_goal   

""" 

#!/usr/bin/env python

# Import Libraries
import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from std_srvs.srv import *
import sys
import select
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2022.msg import Posxy_velxy
from colorama import Fore, Style
from colorama import init
init()

# Subscriber's callback function
def publisher(msg):   
    """
    This function is a callback function that is called whenever 
    a message is received from the *odom* topic. The function 
    extracts the position and velocity information from the 
    message and creates a custom message that contains these 
    parameters.
    
    The function then publishes the custom message to the
    *posxy_velxy* topic.
    
    """
                       
    global pub
    posi = msg.pose.pose.position             # get the position information from the msg
    velo = msg.twist.twist.linear             # get the velocity information from the msg
    posxy_velxy = Posxy_velxy()               # create custom message
    
    # set the custom message's parameters
    
    posxy_velxy.msg_pos_x = posi.x
    posxy_velxy.msg_pos_y = posi.y
    posxy_velxy.msg_vel_x = velo.x
    posxy_velxy.msg_vel_y = velo.y
    pub.publish(posxy_velxy)                  # publish the custom message
 
    
def action_client():
    """
    This function implements the action client. It creates an instance  
    of the *SimpleActionClient* class and waits for the action server  
    to start.
    
    It then enters into a loop that prompts the user to input the target 
    position. If the user enters *C*, the function cancels the current  
    goal if it exists. Otherwise, it converts the user's input to float 
    data types and creates a goal message.
    
    Finally, it sends the goal message to the action server.
    
    """
    
    # create the action client
    action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
    action_client.wait_for_server()           # wait for the server to be started
    status_goal = False

    while not rospy.is_shutdown():
        # Ready the computer keyboard inputs
        print(Fore.WHITE + "Please input the target's position or type C to cancel it ")
        
        # print(Fore.BLUE + "Ideal X Position: ")
        x_posi_input = input(Fore.BLUE + "Ideal X Position: ")
        
        # print(Fore.BLUE + "Ideal Y Position: ")
        y_posi_input = input(Fore.BLUE + "Ideal Y Position: ")
        
 	# If user entered 'c' and the two-wheeled robot is reaching the goal position, just cancel the goal
        if x_posi_input == "c" or y_posi_input == "c":      
            action_client.cancel_goal()      # cancel the goal
            status_goal = False
        else:
            # Convert the data type of the numbers from string to float
            x_posi_conv = float(x_posi_input)
            y_posi_conv = float(y_posi_input)
            
            # Let's create the goal to send to the server
            goal = assignment_2_2022.msg.PlanningGoal()
            goal.target_pose.pose.position.x = x_posi_conv
            goal.target_pose.pose.position.y = y_posi_conv
            action_client.send_goal(goal)                      # send the goal to the action server
            status_goal = True

def main():
    """
    This function is the main function of the script. It initializes 
    the ROS node, creates a publisher *posxy_velxy* and a subscriber
    *odom*. Then, it calls the *action_client()* function.
    
    """
    
    rospy.init_node('node_a_action_client')                    # initialize the node
    global pub                                                 # global publisher
    # publisher: sends a message which contains two parameters (position and velocity) 
    pub = rospy.Publisher("/posxy_velxy", Posxy_velxy, queue_size = 1) 
    # subscriber: get from "Odom" two parameters (position and velocity) 
    sub_from_Odom = rospy.Subscriber("/odom", Odometry, publisher)  
    action_client()                                            # finally, call the function client

if __name__ == '__main__':
    main()
