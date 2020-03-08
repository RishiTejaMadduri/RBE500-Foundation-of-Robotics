#!/usr/bin/env python

# Import required services, messages and libraries
import sys
import rospy
import numpy as np
from rbe_500_homeworks.srv import *
from sensor_msgs.msg import *

# Function for calling forward velocity kinematics service, takes joint positions and joint velocities as input and sends them to the service, returns the service response(end effector velocities)
def vel_kinematics_client(q1,q2,q3,q1_dot,q2_dot,q3_dot):
    rospy.wait_for_service('vel_kinematics')
    try:
        vel_kinematics = rospy.ServiceProxy('vel_kinematics', VelKinematics)
        resp = vel_kinematics(q1,q2,q3,q1_dot,q2_dot,q3_dot)
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
        
# Function for calling inverse velocity kinematics service, takes joint positions and end effector velocities as input and sends them to the service, returns the service response(joint velocities)        
def inv_vel_kinematics_client(q1,q2,q3,v_x,v_y,v_z,w_x,w_y,w_z):
    rospy.wait_for_service('inv_vel_kinematics')
    try:
        inv_vel_kinematics = rospy.ServiceProxy('inv_vel_kinematics', InvVelKinematics)
        resp = inv_vel_kinematics(q1,q2,q3,v_x,v_y,v_z,w_x,w_y,w_z)
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

# Function to get joint position readings from gazebo as sensor messages, returns the joint positions    
def gazebo_get():

    rospy.init_node('gazebo_get', anonymous=True)
    #rospy.Subscriber("/custom_scara/joint_states", JointState, joint_pos)
    #rospy.spin()
    data = rospy.wait_for_message("/custom_scara/joint_states", JointState)
    
    return data.position

# Main function
if __name__ == "__main__":
        
    # Get joint position readings
    q = None
    q = gazebo_get()
    print('\nMeasured Joint Positions: {}\n'.format(q))
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    # Request user input for joint velocities
    q1_dot = float(input('Provide q1_dot: '))
    q2_dot = float(input('Provide q2_dot: '))
    q3_dot = float(input('Provide q3_dot: '))
    # Send readings and inputs to the forward velocity kinematics node and receive the result
    ans = vel_kinematics_client(q1,q2,q3,q1_dot,q2_dot,q3_dot)
    ee_velocity = ans.ee_velocity
    print ("\nRequesting the 6x1 end effector velocity vector...\n")
    print ("6x1 velocity vector [v_x,v_y,v_z,w_x,w_y,w_z]:\n{}".format(ee_velocity))
    # Send forward velocity kinematics result to the inverse velocity kinematics node and receive the result
    ans = inv_vel_kinematics_client(q1,q2,q3,ee_velocity[0],ee_velocity[1],ee_velocity[2],ee_velocity[3],ee_velocity[4],ee_velocity[5])
    joint_vel = ans.q_dot
    print ("\nRequesting joint velocities...\n")
    print ("Joint velocities [q1_dot,q2_dot,q3_dot]:\n{}".format(joint_vel))
