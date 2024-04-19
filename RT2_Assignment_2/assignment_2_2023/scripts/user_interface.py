#! /usr/bin/env python

"""
.. module::nodeA
    
    :platform:Unix
    :synopsys: Python node A for assignment 2 of Research Track 1 course

.. moduleauthor:: Iris Laanearu laanearu.iris@gmail.com

This node implements an action client, allowing the user to set a target (x, y) or to cancel it. It uses the feedback/status of the action server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom.

Subscribes to:
    odom
    
Publishes to: 
    RobotState   
"""


from __future__ import print_function
import rospy
import actionlib
import assignment_2_2023.msg
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from assignment_2_2023.msg import PlanningFeedback, RobotState
   
def set_goal(client, x, y):
    """
    Set the new goal function
    
    This function sets the new goal given by user input and sends the new goal to the action server.
    
    Parameters:
    *client*: action client to send the goal to
    *x*: x coordinate of the goal
    *y*: y coordinate of the goal
    """
    goal = assignment_2_2023.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal)

def cancel_goal(client):
    """
    Cancel the goal function
    
    This function cancels an active goal when user indicates this action and sends the cancel goal message to the action server.
    
    Parameters:
    *client*: action client to send the cancel goal message to
    """
    if client and client.get_state() == GoalStatus.ACTIVE:
        print("Cancelling goal...")
        client.cancel_goal()
        client.wait_for_result()  # Wait for the cancellation to be processed
        return True
    else:
        print("No active goal to cancel.")
        return False

# Callback function for the /odom topic 
def odom_callback(odom_msg, robot_state_publisher):
    """
    Callback function for the /odom topic
    
    This is a callback function to extract position and velocity of the robot from the odom topic and publishes the position and velocity to the custom RobotState topic.
    
    Parameters:
    *odom_msg*: odom topic 
    *robot_state_publisher*: RobotState topic
    """
    # Extract position and velocity of the robot from the odom message
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    vel_x = odom_msg.twist.twist.linear.x
    vel_z = odom_msg.twist.twist.angular.z

    # Publish the position and velocity on the RobotState topic
    robot_state_msg = RobotState(x=x, y=y, vel_x=vel_x, vel_z=vel_z) # Write the message
    robot_state_publisher.publish(robot_state_msg) # Publish the message       
        
def main():
    """
    Main function of the node A
    
    In the main function the node is initalized and the action client is created. Also publishers and subscribers are defined. A loop is created that asks the user for a new goal coordinates and once the goal has been set then it gives the user an opportunity to cancel the active goal. Additonally, it updates the user of the status of the goal.
    
    """
    # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS
    rospy.init_node('user_interface')
    # Create the SimpleActionClient
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    # Wait until the action server has started up and started listening for goals
    client.wait_for_server()
    
    # Publisher to the RobotState custom message
    robot_state_publisher = rospy.Publisher('/RobotState', RobotState, queue_size=1)
    """ Publisher to the custom topic RobotState
    """
    
    # Subscribe to the /odom topic
    odom_subscriber = rospy.Subscriber('/odom', Odometry, odom_callback, robot_state_publisher, queue_size=1)
    """ Subscriber to the odom topic
    """
    
    while not rospy.is_shutdown():
        rospy.sleep(1)  # Give some time so asking for goal would be the last
    	
        # User enters the goal coordinates
        x = float(input("Enter x coordinate for the goal: "))
        y = float(input("Enter y coordinate for the goal: "))

        # Send the goal
        set_goal(client, x, y)
        
        # Continuously check for status of the goal
        while not rospy.is_shutdown():
            rospy.sleep(1)  # Give some time for the goal to be processed
	    
	    # Check the status of the goal		
            goal_status = client.get_state()
		
            # If the goal is active
            if goal_status == GoalStatus.ACTIVE:
                print("Goal is active.")
                
                # Ask if the user wants to cancel the goal
                cancel_input = input("Do you want to cancel the goal? (yes/no): ").lower()
                if cancel_input == "yes":
                    success = cancel_goal(client)
                    # Check if the goal has been canceled successfully
                    if success:
                        print("Goal successfully canceled.")
                        break
                    else:
                        print("Failed to cancel goal.")
                        break  
            # If the goal has been reached            
            elif goal_status == GoalStatus.SUCCEEDED:
                print("Goal is reached.")
                break
            # If the goal has been canceled    
            elif goal_status == GoalStatus.ABORTED:
                print("Goal is canceled.")
                break
            # If the goal is not active    
            elif goal_status != GoalStatus.ACTIVE:
                print("Failed to set the goal.")  
                break  
                 
        # Set up the new goal    
        print("Set a new goal.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
        pass

