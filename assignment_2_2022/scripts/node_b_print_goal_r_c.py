"""
.. module:: node_b_print_goal_r_c
   :platform: Unix
   :synopsis: Python module responsible to print the number of goal reached and cancelled
   
.. moduleauthor:: Natnael Berhanu Takele

This node implements a ROS service node. The code receives messages from a topic and a service, and responds with the count of goals that were cancelled and reached. 

The script creates a service that listens to the *reaching_goal* result topic and counts the number of goals that have been cancelled and reached.

Subscribes to:
   /reaching_goal/result

""" 

#! /usr/bin/env python

# Import Libraries
import rospy 
from assignment_2_2022.srv import goal_rc, goal_rcResponse 
import actionlib   
import actionlib.msg  
import assignment_2_2022.msg  

class Service:
    def __init__(self):
        """
        This function:
    
          - initializes the counters for the reached and cancelled goals
          - creates the service
          - subscribes to the result topic
          
        """
           
        # Initialize the counters for reached and cancelled goals
        self.reached_goal = 0
        self.cancelled_goal = 0
        
        # Create the service
        self.srv = rospy.Service('node_b_print_goal_r_c', goal_rc, self.data) 
        
        # Subscribes to the result topic
        self.sub_result = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, self.outcome_callback)
        
    def outcome_callback(self, msg):
        """
        This function subscribes to the *result* topic, obtains  
        the status of the message,and increments the appropriate 
        counter based on whether the goal was cancelled or reached.
        
        """
        status = msg.status.status             # Obtain the result's status from the msg
        
        # Cancelled goal (status = 2)
        if status == 2:
            self.cancelled_goal += 1
        # Reached goal (status = 3)
        elif status == 3:
            self.reached_goal += 1
    def data(self, req):
        """
        This function sends back a message with the most recent 
        values for the goals that were cancelled and reached.
         
        """  
        return goal_rcResponse(self.reached_goal, self.cancelled_goal)
        
def main():
    """
    This function:
    
          - initializes the node
          - creates an instance of the *Service* class
          - awaits messages
    
    """
    rospy.init_node('node_b_print_goal_r_c')          # Initialize the node
    node_b_print_goal_r_c = Service()                 # Create an instance of the Service class
    rospy.spin()                                      # Await messages
if __name__ == "__main__":
    main()
