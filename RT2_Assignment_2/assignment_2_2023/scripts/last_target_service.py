#!/usr/bin/env python3

"""
Module: nodeB
    
Platform: Unix
Synopsis: Python node B for assignment 2 of Research Track 1 course

Author: Iris Laanearu (laanearu.iris@gmail.com)

A service node that, when called, returns the coordinates of the last target sent by the user.

Subscribes to:
    /reaching_goal/goal  
"""

import rospy
from assignment_2_2023.msg import PlanningActionGoal
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolResponse

# Global variables to store the last target coordinates
last_target = Point()
last_target.x = rospy.get_param('des_pos_x', default=0.0)  
last_target.y = rospy.get_param('des_pos_y', default=0.0)  
last_target.z = 0

# Service active state
active_ = False

# Service callback function
def last_target_handler(req):
    """
    Last target service callback function
    
    This is a callback function to print the coordinates of the last target sent to the user. This service is started when it is called with parameter True.
    
    Parameters:
    - req: request parameter to activate the service
    """
    global last_target, active_

    # Print the last target coordinates to the console
    print(f"\nLast Target set by the user: x = {last_target.x:.4f}, y = {last_target.y:.4f}")

    # Return the status of the service call
    active_ = req.data
    response = SetBoolResponse()
    response.success = True
    response.message = 'Done!'
    return response

# Callback function for the reaching_goal/goal topic
def goal_callback(msg):
    """
    Reaching_goal/goal topic callback function
    
    This is a callback function to extract position of the last target sent by the user from the reaching_goal/goal topic and save it to the global variable last_target.
    
    Parameters:
    - msg: reaching_goal/goal message
    """
    global last_target
    last_target.x = msg.goal.target_pose.pose.position.x
    last_target.y = msg.goal.target_pose.pose.position.y 

def main():
    """
    Main function of node B
    
    In the main function the node is initialized and subscriber is defined. Also the service node is initialized and in case called the the last target sent by the user is printed using the last_target_handler function. This will run until ROS is shut down.
    
    """
    # Initialize the node
    rospy.init_node('last_target')

    # Subscribe to the reaching_goal/goal topic
    sub_goal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)

    # Create the service
    rospy.Service('last_target', SetBool, last_target_handler)

    rospy.spin()

if __name__ == '__main__':
    main()

