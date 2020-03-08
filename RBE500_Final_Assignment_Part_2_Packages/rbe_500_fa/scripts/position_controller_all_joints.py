#!/usr/bin/env python

# Import required services, messages and libraries
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from sensor_msgs.msg import *

def set_joint_effort(joint,effort,start,duration):
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    try:
        set_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        success = set_effort(joint,effort,start,duration)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def clear_joint_effort(joint):
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    try:
        clear_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        success = clear_effort(joint)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Function to get joint position readings from gazebo as sensor messages, returns the joint positions
def gazebo_get():

    rospy.init_node('gazebo_get', anonymous=True)
    data = rospy.wait_for_message("/custom_scara/joint_states", JointState)
    return data.position

# Main function
if __name__ == "__main__":

    # Get joint position readings
    q = None
    q_raw = gazebo_get()
    q = np.array(q_raw).reshape((3,1))
    duration = rospy.Time(1)
    #duration.secs = -0.1
    start = rospy.Time(0)
    K_p = [200,100,420]
    K_d = [20,10,15]
    for j in range(0,len(q)):
        time_init = rospy.get_time()
        time_old = time_init
        e_dot = 0
        e_int = 0
        reference = float(input("Reference joint position: "))
        e = reference - q[j]
        e_old = e
        ref_rec = list()
        pos_rec = list()
        time_rec = list()
        joint = 'joint{}'.format(j+1)
        while len(time_rec)<150:
            q = gazebo_get()
            time = rospy.get_time()
            e = reference - q[j]
            e_dot = (e - e_old)/(time - time_old)
            e_int = e_int + e *(time - time_old)
            effort = K_p[j]*e + K_d[j]*e_dot #+ K_i*e_int
            clear_success = clear_joint_effort(joint)
            effort_success = set_joint_effort(joint,effort,start,duration)
            time_old = time
            e_old = e
            print("error: {}".format(e))
            ref_rec.append(reference)
            pos_rec.append(q[j])
            time_rec.append(time-time_init)

        plt.figure(j+1)
        plt.plot(time_rec,ref_rec)
        plt.plot(time_rec,pos_rec)
        plt.ylim(0,1.5)
        plt.grid(b=True)

    plt.show()
    # while len(time_rec)<150:
    #     q_raw = gazebo_get()
    #     q = np.array(q_raw).reshape((3,1))
    #     time = rospy.get_time()
    #     e = reference - q
    #     e_dot = (e - e_old)/(time - time_old)
    #     e_int = e_int + e *(time - time_old)
    #     K_p = np.diag([600,500,420])
    #     K_d = np.diag([30,25,15])
    #     K_i = 1
    #     effort = np.matmul(K_p,e) + np.matmul(K_d,e_dot) #+ K_i*e_int
    #     for i in range(0,len(q)):
    #         joint = 'joint{}'.format(i+1)
    #         clear_success = clear_joint_effort(joint)
    #     for i in range(0,len(q)):
    #         joint = 'joint{}'.format(i+1)
    #         effort_success = set_joint_effort(joint,effort[i],start,duration)
    #     time_old = time
    #     e_old = e
    #     print("error: {}".format(e))
    #     ref_rec.append(reference)
    #     pos_rec.append(q[0])
    #     time_rec.append(time-time_init)
    #
    # plt.plot(time_rec,ref_rec)
    # plt.plot(time_rec,pos_rec)
    # plt.ylim(0,1.5)
    # plt.grid(b=True)
    # plt.show()
    #
    # zip(time_rec,ref_rec,pos_rec)
    # import csv
    # with open('recorded.csv', 'w') as f:
    #     writer = csv.writer(f, delimiter='\t')
    #     writer.writerows(zip(time_rec,ref_rec,pos_rec))
    #
    # quit()
