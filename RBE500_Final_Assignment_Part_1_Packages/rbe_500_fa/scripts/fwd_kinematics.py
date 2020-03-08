#!/usr/bin/env python

# Import required services and libraries
from rbe_500_fa.srv import FwdKinematics,FwdKinematicsResponse
import rospy
import numpy as np
import math

# Function for handling forward kinematics, takes joint positions as input (as service request)
def handle_fwd_kinematics(req):
    
    # Reassign joint positions
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3
    # Initialize service response
    resp = FwdKinematicsResponse()
    # Link lengths of the robot !!Should be corrected!!
    l1 = 2
    l2 = 1
    l3 = 1.5
    l4 = 0.5
    # DH Parameters of the Robot
    theta = [q1,q2,0]
    d = [l1,0,q3+l4]
    a = [l2,l3,0]
    alpha = [0,np.pi,0]
    
    print ("\nProvided values are:\ntheta_1={} rads\ntheta_2={} rads\nd_3={} m\n".format(q1,q2,q3))
    print ("Returning 6x1 pose vector...\n")
    # Homogenous transformation matrices
    T_1_0 = np.array([[np.cos(theta[0]), -np.sin(theta[0])*np.cos(alpha[0]),  np.sin(theta[0])*np.sin(alpha[0]), a[0]*np.cos(theta[0])],
                      [np.sin(theta[0]),  np.cos(theta[0])*np.cos(alpha[0]), -np.cos(theta[0])*np.sin(alpha[0]), a[0]*np.sin(theta[0])],
                      [               0,                   np.sin(alpha[0]),                   np.cos(alpha[0]),                  d[0]],
                      [               0,                                  0,                                  0,                     1]])
    
    T_2_1 = np.array([[np.cos(theta[1]), -np.sin(theta[1])*np.cos(alpha[1]),  np.sin(theta[1])*np.sin(alpha[1]), a[1]*np.cos(theta[1])],
                      [np.sin(theta[1]),  np.cos(theta[1])*np.cos(alpha[1]), -np.cos(theta[1])*np.sin(alpha[1]), a[1]*np.sin(theta[1])],
                      [               0,                   np.sin(alpha[1]),                   np.cos(alpha[1]),                  d[1]],
                      [               0,                                  0,                                  0,                     1]])
    
    T_3_2 = np.array([[np.cos(theta[2]), -np.sin(theta[2])*np.cos(alpha[2]),  np.sin(theta[2])*np.sin(alpha[2]), a[2]*np.cos(theta[2])],
                      [np.sin(theta[2]),  np.cos(theta[2])*np.cos(alpha[2]), -np.cos(theta[2])*np.sin(alpha[2]), a[2]*np.sin(theta[2])],
                      [               0,                   np.sin(alpha[2]),                   np.cos(alpha[2]),                  d[2]],
                      [               0,                                  0,                                  0,                     1]])
    
    # Forward kinematics result, 4x4 transformation matrix from tip to base
    T_3_0 = np.matmul(T_1_0,np.matmul(T_2_1,T_3_2))
    # Generating service response as the 6x1 vector, including the end effector position and orientation in ZYX Euler Angles
    resp.ee_pos_or = np.zeros((6))
    resp.ee_pos_or[0:3] = T_3_0[0:3,3]
    beta = math.atan2(-T_3_0[2,0],math.sqrt(T_3_0[0,0]**2 + T_3_0[1,0]**2))
    alpha = math.atan2(T_3_0[0,1]/math.cos(beta),T_3_0[0,0]/math.cos(beta))
    gamma = math.atan2(T_3_0[2,1]/math.cos(beta),T_3_0[2,2]/math.cos(beta))
    resp.ee_pos_or[3] = alpha
    resp.ee_pos_or[4] = beta
    resp.ee_pos_or[5] = gamma
    return resp

# Forward kinematics server
def fwd_kinematics_server():
    rospy.init_node('fwd_kinematics_server')
    # Establish the service that produces response by handle_fwd_kinematics function
    s = rospy.Service('fwd_kinematics', FwdKinematics, handle_fwd_kinematics)
    rospy.spin()
    
# Main function
if __name__ == "__main__":
    fwd_kinematics_server()
