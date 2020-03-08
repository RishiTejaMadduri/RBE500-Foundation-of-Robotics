#!/usr/bin/env python

# Import required services and libraries
from rbe_500_fa.srv import InvKinematics,InvKinematicsResponse
import rospy
import numpy as np
import math

# Function to handle inverse kinematics, takes 6x1 vector for end effector position and orientation in ZYZ Euler Angles, returns the joint positions
def handle_inv_kinematics(req):
    
    # Reassign coordinates and Euler angles
    x = req.ee_pos_or[0]
    y = req.ee_pos_or[1]
    z = req.ee_pos_or[2]
    alpha = req.ee_pos_or[3]
    beta = req.ee_pos_or[4]
    gamma = req.ee_pos_or[5]
    # Initialize service response
    resp = InvKinematicsResponse()
    # Link lengths of the robot !!Should be corrected!!
    l1 = 2
    l2 = 1
    l3 = 1.5
    l4 = 0.5
    print ("\nProvided pose vector is :\nx={}\ny={}\nz={}\nalpha={}\nbeta={}\ngamma={}\n".format(x,y,z,alpha,beta,gamma))
    print ("Returning joint variables...\n")
    # Inverse kinematics calculations
    q3 = l1 - l4 -z
    
    q2_guess = math.pi - math.acos(-(x**2+y**2-l2**2-l3**2)/(2*l2*l3))
    q1_guess = math.atan2(y,x) - math.atan2(l3*math.sin(q2_guess),l2+l3*math.cos(q2_guess))
    phi = math.acos(math.cos(alpha)*math.cos(beta))
    eps = 0.01
    if abs((q2_guess + q1_guess) - phi) > eps:
        q2 = -q2_guess
        q1 = math.atan2(y,x) - math.atan2(l3*math.sin(q2),l2+l3*math.cos(q2))
    else:
        q2 = q2_guess
        q1 = q1_guess
    resp.q = [q1,q2,q3]
    return resp

# Inverse kinematics server
def inv_kinematics_server():
    rospy.init_node('inv_kinematics_server')
    # Establish the service that produces response by handle_inv_kinematics function
    s = rospy.Service('inv_kinematics', InvKinematics, handle_inv_kinematics)
    rospy.spin()
    
# Main function        
if __name__ == "__main__":
    inv_kinematics_server()
