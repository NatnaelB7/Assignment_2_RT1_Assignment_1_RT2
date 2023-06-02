"""
.. module:: node_c_print_dist_spd
   :platform: Unix
   :synopsis: Python module responsible to print out the distance and speed data
   
.. moduleauthor:: Natnael Berhanu Takele

This node subscribes to a ROS topic *posxy_velxy* (published by :mod:`node_a_action_client`) that publishes messages with the two-wheeled robot's position and velocity. It then calculates the distance between the robot's position and a desired position, as well as the average speed of the robot. 

Moreover, the robot's average speed and distance from the target are printed out by using this node. These parameters are taken from the *posxy_velxy* topic as a custom message.

Subscribes to:
   /posxy_velxy
   
ROS parameter:
   desired_pos_x
   desired_pos_y
   
""" 

#! /usr/bin/env python

# Import Libraries
import rospy
import math
import time
from assignment_2_2022.msg import Posxy_velxy
from colorama import init
init()
from colorama import Fore, Back, Style

class Print_Dist_Spd:
    def __init__(self):
        """
        This is the constructor method of the *Print_Dist_Spd* class.
        It initializes some variables and sets up a subscriber to 
        listen to the *posxy_velxy* topic.
        
        """
        self.F = rospy.get_param("frequency")      # Obtain the parameter of publish frequency 
        self.recent_time = 0                       # The most recent printing of the information
        
        # Subscriber to the topic of position and velocity 
        self.sub_pos = rospy.Subscriber("/posxy_velxy", Posxy_velxy, self.outcome_callback)

    def outcome_callback(self, msg):
        """
        This is the callback function that gets called each time 
        a new message is received on the *posxy_velxy* topic. 
        
        It extracts the necessary information from the message
        and calculates the distance and average speed. Then, it
        updates the value of robot current velocity and distance 
        from target.
        
        Finally, it prints out the distance and speed information.
        
        """    
        T = (1.0/self.F) * 1000                   # Calculate the duration in milli-seconds
        current_time = time.time() * 1000         # Calculate the current time in milli-seconds

 # Verify whether the difference between the current time and the most recent printed time is larger than the period
        if current_time - self.recent_time > T:
            # Determine the desired position using the ROS parameters
            x_target = rospy.get_param("desired_pos_x")
            y_target = rospy.get_param("desired_pos_y")
            
            # Using the message, determine the two-wheeled robot's actual position
            x_2w_robot = msg.msg_pos_x
            y_2w_robot = msg.msg_pos_y
            
            # Compute the distance between the desired and actual positions
            distance = round(math.dist([x_target, y_target], [x_2w_robot, y_2w_robot]),2)
            
            # Using the message, determine the two-wheeled robot's actual velocity 
            vel_x = msg.msg_vel_x
            vel_y = msg.msg_vel_y      
                 
            # Determine the average speed using the velocity components from the message
            avg_spd = round(math.sqrt(vel_x**2 + vel_y**2),2)
            
            # Print out the distance and speed data
            print(Fore.BLUE + "The Distance of the Two-Wheeled Robot from the Target: %s [m]", distance)
            print(Fore.RED + "The Average Speed of the Two-Wheeled Robot: %s [m/s]", avg_spd)
            
            # Update the time that was previously displayed
            self.recent_time = current_time

def main():
    """
    This function: 
         - initializes the ROS node
         - creates an instance of the *Print_Dist_Spd* class
         - enters into a loop to wait for messages
    
    """
    rospy.init_node('node_c_print_dist_spd')         # Initialize the node
    node_c_print_dist_spd = Print_Dist_Spd()         # Create an instance of the Print_Dist_Spd class
    rospy.spin()                                     # Await messages

if __name__ == "__main__":
    main()
