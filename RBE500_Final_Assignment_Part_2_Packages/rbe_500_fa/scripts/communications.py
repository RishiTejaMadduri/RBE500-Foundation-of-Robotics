#!/usr/bin/env python

# Import required services, messages and libraries
import sys
import rospy
import numpy as np
from rbe_500_fa.srv import *
from sensor_msgs.msg import *

# Function for calling forward kinematics service, takes joint positions as input and sends them to the service, returns the service response(end effector position and orientation)
def fwd_kinematics_client(q1,q2,q3):
    rospy.wait_for_service('fwd_kinematics')
    try:
        fwd_kinematics = rospy.ServiceProxy('fwd_kinematics', FwdKinematics)
        resp = fwd_kinematics(q1,q2,q3)
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

# Function for calling inverse kinematics service, takes end effector position and orientation as input and sends them to the service, returns the service response(joint positions)
def inv_kinematics_client(ee_pos_or):
    rospy.wait_for_service('inv_kinematics')
    try:
        inv_kinematics = rospy.ServiceProxy('inv_kinematics', InvKinematics)
        resp = inv_kinematics(ee_pos_or)
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
    
# Function to get joint position readings from gazebo as sensor messages, returns the joint positions
def gazebo_get():
    rospy.init_node('gazebo_get', anonymous=True)
    data = rospy.wait_for_message("/custom_scara/joint_states", JointState)
    return data.position

# Main function
if __name__ == "__main__":
    
    # Get joint position readings
    q = None
    q = gazebo_get()
    print('\nMeasured Joint Positions:[q1,q2,q3]\n {}\n'.format(np.round(q,3)))
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    # Send readings to the forward kinematics node and receive the result
    ans = fwd_kinematics_client(q1,q2,q3)
    ee_pos_or = ans.ee_pos_or
    print ("\nRequesting the 6x1 pose vector...\n")
    print ("6x1 pose vector [x,y,z,alpha,beta,gamma]:\n{}".format(np.round(ee_pos_or,3)))
    # Send forward kinematics result to the inverse kinematics node and receive the result
    ans = inv_kinematics_client(ee_pos_or)
    joints = ans.q
    print ("\nRequesting joint variables...\n")
    print ("Joint variables [q1,q2,q3]:\n{}".format(np.round(joints,3)))
    print ("\nDifference between measured and computed values:\n")
    print('\nFor q1: {}\n'.format(np.round(q[0]-joints[0],3)))
    print('For q2: {}\n'.format(np.round(q[1]-joints[1],3)))
    print('For q3: {}\n'.format(np.round(q[2]-joints[2],3)))
