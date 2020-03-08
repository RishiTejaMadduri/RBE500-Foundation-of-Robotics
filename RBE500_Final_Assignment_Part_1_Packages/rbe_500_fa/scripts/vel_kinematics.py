#!/usr/bin/env python

# Import required services and libraries
from rbe_500_homeworks.srv import VelKinematics,VelKinematicsResponse, InvVelKinematics,InvVelKinematicsResponse
import rospy
import numpy as np
import math

# Function to compute the jacobian, takes joint positions as input, returns the jacobian matrix
def generate_jacobian(q):
    
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    
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
    # Intermediate transformation required to compute jacobian
    T_2_0 = np.matmul(T_1_0,T_2_1)
    # z-axes of the frames
    z_0_0 = np.array([0,0,1])
    z_1_0 = T_1_0[0:3,2]
    z_2_0 = T_2_0[0:3,2]
    # origins of the frams
    o_0_0 = np.array([0,0,0])
    o_1_0 = T_1_0[0:3,3]
    o_2_0 = T_2_0[0:3,3]
    o_3_0 = T_3_0[0:3,3]
    # Jacobian computation
    J = np.zeros((6,3))
    J[0:3,0] = np.cross(z_0_0,np.subtract(o_3_0,o_0_0))
    J[0:3,1] = np.cross(z_1_0,np.subtract(o_3_0,o_1_0))
    J[0:3,2] = z_2_0
    J[3:,0] = z_0_0
    J[3:,1] = z_1_0
    J[3:,2] = np.array([0,0,0])
    
    return J

# Function to handle forward velocity kinematics, takes joint positions and joint velocities as input, returns the cartesian end effector velocities
def handle_vel_kinematics(req):
    
    # Reassign joint positions and velocities
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3

    q1_dot = req.q1_dot
    q2_dot = req.q2_dot
    q3_dot = req.q3_dot
    
    print ("\nProvided values are:\ntheta_1={} rads\ntheta_2={} rads\nd_3={} m\ntheta_1_dot={} \ntheta_2_dot={} \nd_3_dot={} \n".format(q1,q2,q3,q1_dot,q2_dot,q3_dot))
    print ("Returning 6x1 velocity vector...\n")
    
    # Initialize service response
    resp = VelKinematicsResponse()
    
    # Request jacobian from the function
    J = generate_jacobian([q1,q2,q3])
    
    # Generate the vector for joint velocities
    q_dot = np.array([q1_dot,q2_dot,q3_dot]).reshape((3,1))
    #ee_velocity = np.zeros((6))
    # Compute the end effector velocities
    ee_velocity = np.matmul(J,q_dot)
    resp.ee_velocity = ee_velocity
    
    return resp
    
# Function to handle inverse velocity kinematics, takes joint positions and cartesian end effector velocities as input, returns the joint velocities
def handle_inv_vel_kinematics(req):
    
    # Reassign joint positions and end effector velocities
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3

    v_x = req.v_x
    v_y = req.v_y
    v_z = req.v_z
    w_x = req.w_x
    w_y = req.w_y
    w_z = req.w_z
    
    print ("\nProvided values are:\ntheta_1={} rads\ntheta_2={} rads\nd_3={} m\nv_x={} \nv_y={} \nv_z={} \nw_x={} \nw_y={} \nw_z={} \n".format(q1,q2,q3,v_x,v_y,v_z,w_x,w_y,w_z))
    print ("Returning 6x1 velocity vector...\n")
    
    # Initialize service response
    resp = InvVelKinematicsResponse()
    
    # Request jacobian from the function
    J = generate_jacobian([q1,q2,q3])
    
    # Compute inverse jacobian
    J_inv = np.linalg.pinv(J)
    
    # Generate the vector for end effector velocities
    ee_velocity = np.array([v_x,v_y,v_z,w_x,w_y,w_z]).reshape((6,1))
    #q_dot = np.zeros((3))
    # Compute the joint velocities
    q_dot = np.matmul(J_inv,ee_velocity)
    resp.q_dot = q_dot
    
    return resp

# Forward velocity kinematics server
def vel_kinematics_server():
    #rospy.init_node('vel_kinematics_server')
    # Establish the service that produces response by handle_vel_kinematics function
    s1 = rospy.Service('vel_kinematics', VelKinematics, handle_vel_kinematics)
    #rospy.spin()

# Inverse velocity kinematics server
def inv_vel_kinematics_server():
    #rospy.init_node('inv_vel_kinematics_server')
    # Establish the service that produces response by handle_inv_vel_kinematics function
    s2 = rospy.Service('inv_vel_kinematics', InvVelKinematics, handle_inv_vel_kinematics)
    #rospy.spin()
    

# Main function     
if __name__ == "__main__":
    rospy.init_node('vel_kinematics')
    vel_kinematics_server()
    inv_vel_kinematics_server()
    rospy.spin()
    
