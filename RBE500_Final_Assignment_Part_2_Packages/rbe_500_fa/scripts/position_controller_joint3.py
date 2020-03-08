#!/usr/bin/env python
# Import required services, messages and libraries
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from sensor_msgs.msg import *

# Function to set joint efforts through the service '/gazebo/apply_joint_effort'
# Takes in the joint name, effort, start time and duration as input arguments.
# Returns 1 for successful application.
def set_joint_effort(joint,effort,start,duration):
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    try:
        set_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        success = set_effort(joint,effort,start,duration)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Function to clear joint efforts through the service '/gazebo/clear_joint_forces'
# Takes in the joint name and returns 1 for successful application.
def clear_joint_effort(joint):
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    try:
        clear_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        success = clear_effort(joint)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Function to get joint position readings from gazebo as sensor messages, returns the joint positions.
def gazebo_get():

    rospy.init_node('gazebo_get', anonymous=True)
    data = rospy.wait_for_message("/custom_scara/joint_states", JointState)
    return data.position

# Main function
if __name__ == "__main__":

    # Get joint position readings
    q = None
    q = gazebo_get()
    # Print out the initial joint position in terminal:
    print("Initial position of the 3rd joint: {}".format(np.round(q[0],4)))
    # Define duration and start time to input to set_joint_effort function
    duration = rospy.Time(1)
    start = rospy.Time(0)
    # Initial time is set as the current time
    time_init = rospy.get_time()
    time_old = time_init
    # e_dot initialized
    e_dot = 0
    # integral term can be used if PID desired:
    # e_int = 0
    # Get user input for joint position reference
    reference = float(input("Reference joint position between 0-1.5: "))
    # Compute error
    e = reference - q[0]
    e_old = e
    # Initialize lists to store time, reference and actual position
    ref_rec = list()
    pos_rec = list()
    time_rec = list()
    # While loop for the controller (runs for a predefined time):
    while len(time_rec)<150:
        # Get joint position:
        q = gazebo_get()
        # Get current time:
        time = rospy.get_time()
        # Compute error:
        e = reference - q[0]
        # Compute e_dot:
        e_dot = (e - e_old)/(time - time_old)
        # Integral of the error, can be used in PID:
        # e_int = e_int + e *(time - time_old)
        # Proportional controller gain
        K_p = 475
        # Derivative controller gain
        K_d = 35
        # Integral controller gain, can be used in PID:
        # K_i = 1
        # Compute effort:
        effort = K_p*e + K_d*e_dot #+ K_i*e_int, add integral term for PID
        # Clear previous joint efforts:
        clear_success = clear_joint_effort('joint3')
        # Apply the computed joint effort:
        effort_success = set_joint_effort('joint3',effort,start,duration)
        # Store previous time and error
        time_old = time
        e_old = e
        # Print out current joint position and error:
        print("Current position of the 3rd joint: {}".format(np.round(q[0],4)))
        print("error: {}".format(np.round(e,4)))
        # Record the time, reference and actual position:
        ref_rec.append(reference)
        pos_rec.append(q[0])
        time_rec.append(time-time_init)

    # Code to generate plots after running the node:
    plt.plot(time_rec,ref_rec)
    plt.plot(time_rec,pos_rec)
    plt.ylim(0,1.5)
    plt.grid(b=True)
    plt.show()

    # Exporting recorded data to .csv file:
    zip(time_rec,ref_rec,pos_rec)
    import csv
    with open('recorded.csv', 'w') as f:
        writer = csv.writer(f, delimiter='\t')
        writer.writerows(zip(time_rec,ref_rec,pos_rec))

    quit()
