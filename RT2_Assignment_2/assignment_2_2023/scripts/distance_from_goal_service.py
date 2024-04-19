#! /usr/bin/env python3

"""
.. module::nodeC
    
    :platform:Unix
    :synopsys: Python node C for assignment 2 of Research Track 1 course

.. moduleauthor::Iris Laanearu laanearu.iris@gmail.com

A service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.

Subscribes to:
    /reaching_goal/goal
    
Publishes to: 
    RobotState   
"""

import rospy
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import PlanningActionGoal, RobotState
from geometry_msgs.msg import Point, Pose, Twist
from std_srvs.srv import *
import math

# Global variables
# Goal initialization
goal = Point()
goal.x = rospy.get_param('des_pos_x')
goal.y = rospy.get_param('des_pos_y')
goal.z = 0
# Current position initialization
position = RobotState()
# Sevice active state
active = False
# Velocity list to hold previous data
velocities = []

# Get the averaging window size parameter from the launch file
avg_window_size = rospy.get_param('avg_window_size')

# Callbacks
# Callback from the RobotState topic
def state_callback(msg):
    """
    Topic RobotState callback function
    
    This is a callback function to extract position and velocity of the robot from the RobotState topic and calculate the average velocity using th averaging window.
    
    Parameters:
    *msg*: RobotState message
    """
    global position, velocities, avg_window_size
    # Extract position and velocity information from the RobotState message
    position.x = msg.x
    position.y = msg.y
    position.vel_x = msg.vel_x
    position.vel_z = msg.vel_z
    
    # Save the velocity to the list
    velocities.append(position.vel_x)
    # Keep only the last samples of the robot's velocity
    if len(velocities) > avg_window_size:
    	velocities.pop(0) # Remove the oldest velocity in the list

# Callback from the reaching_goal/goal topic    
def goal_callback(msg):
    """
    Topic reaching_goal callback function
    
    This is a callback function to extract the current goal position (x and y) and save it to the global variable goal.
    
    Parameters:
    *msg*: reaching_goal/goal message
    """
    global goal
    goal.x = msg.goal.target_pose.pose.position.x
    goal.y = msg.goal.target_pose.pose.position.y     
    
# Service callback    
def distance_from_goal_handler(req):
    """
    Distance from goal callback
    
    This a service callback function to extract the current goal position (x and y) and save it to the global variable goal. This service is started when it is called with parameter True.
    
    Parameters:
    *req*: request parameter to activate the service
    """
    global goal, position, velocities, active
    
    # Declare distance as a local variable
    distance = Point()
    
    # Calculate distance from the goal
    distance.x = goal.x - position.x
    distance.y = goal.y - position.y
    # Print the distance from the goal
    print(f"\nDistance from the goal: x = {distance.x:.4f}, y = {distance.y:.4f}")
    
    # Calculate the average speed
    # Check if there are values in the velocities list
    if velocities:
        avg_speed = sum(map(abs, velocities)) / len(velocities)
    else: 
        avg_speed = 0.0		
    # Print the average speed
    print(f"Average speed: v = {avg_speed:.4f}")

    # Return the status of the service call
    active = req.data
    response = SetBoolResponse()
    response.success = True
    response.message = 'Done!'
    return response    

def main():
    """
    Main function of node C
    
    In the main function the node is initalized and subscribers are defined. Also the service node is initialized and in case called the distance from goal is printed to the user using the distance_from_goal_handler function. This will run until ROS is shut down.
    
    """
    # Initialize the node	
    rospy.init_node('distance_from_goal')
    
    # Subscribe to the RobotState topic
    sub_odom = rospy.Subscriber('/RobotState', RobotState, state_callback)
    """ Subscriber to the RobotState topic
    """
    
    # Subscribe to the reaching_goal/goal topic
    sub_goal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)
    """ Subscriber to the reaching_goal/goal topic
    """
    
    # Initialize the service
    srv = rospy.Service('distance_from_goal', SetBool, distance_from_goal_handler)
    """ Initialize ROS service
    """
    
    rospy.spin()

if __name__ == '__main__':
    main()
