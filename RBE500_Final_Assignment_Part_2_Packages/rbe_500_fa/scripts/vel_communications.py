#!/usr/bin/env python

# Import required services, messages and libraries
import sys
import rospy
import numpy as np
from rbe_500_fa.srv import *
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
    q_raw = gazebo_get()
    q = np.array(q_raw).reshape((3,1))
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q_old = q
    y_velocity = float(input("Velocity in +y direction: "))
    vel_ref = np.array([0,y_velocity,0,0,0,0]).reshape((6,1))
    duration = rospy.Time(1)
    #duration.secs = -0.1
    start = rospy.Time(0)
    time_init = rospy.get_time()
    time_old = time_init
    e_dot = np.zeros((3,1))
    q_dot = np.array([0,0,0]).reshape((3,1))
    ans = inv_vel_kinematics_client(q1,q2,q3,vel_ref[0],vel_ref[1],vel_ref[2],vel_ref[3],vel_ref[4],vel_ref[5])
    ref_q_dot = np.array(ans.q_dot).reshape((3,1))
    e = ref_q_dot - q_dot
    e_old = e
    ref_rec = list()
    pos_rec = list()
    time_rec = list()
    while len(time_rec) < 250:
        q_raw = gazebo_get()
        q = np.array(q_raw).reshape((3,1))
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        time = rospy.get_time()
        q_dot = (q - q_old)/(time - time_old)
        ans = inv_vel_kinematics_client(q1,q2,q3,vel_ref[0],vel_ref[1],vel_ref[2],vel_ref[3],vel_ref[4],vel_ref[5])
        ref_q_dot = np.array(ans.q_dot).reshape((3,1))
        e = ref_q_dot - q_dot
        print(e)
        e_dot = (e - e_old)/(time - time_old)
        K_p = np.diag([250,250,250])
        K_d = np.diag([1,1,1])
        effort = np.matmul(K_p,e) + np.matmul(K_d,e_dot)
        print(effort)
        for i in range(0,len(q)):
            joint = 'joint{}'.format(i+1)
            clear_success = clear_joint_effort(joint)
        for i in range(0,len(q)):
            joint = 'joint{}'.format(i+1)
            effort_success = set_joint_effort(joint,effort[i],start,duration)
        time_old = time
        e_old = e

    # ref_q_dot = inv_vel_kinematics_client(q1,q2,q3,vel_ref[0],vel_ref[1],vel_ref[2],vel_ref[3],vel_ref[4],vel_ref[5])
    # e = ref_q_dot - q_dot
    # # Request user input for joint velocities
    # q1_dot = float(input('Provide q1_dot: '))
    # q2_dot = float(input('Provide q2_dot: '))
    # q3_dot = float(input('Provide q3_dot: '))
    # # Send readings and inputs to the forward velocity kinematics node and receive the result
    # ans = vel_kinematics_client(q1,q2,q3,q1_dot,q2_dot,q3_dot)
    # ee_velocity = ans.ee_velocity
    # print ("\nRequesting the 6x1 end effector velocity vector...\n")
    # print ("6x1 velocity vector [v_x,v_y,v_z,w_x,w_y,w_z]:\n{}".format(ee_velocity))
    # # Send forward velocity kinematics result to the inverse velocity kinematics node and receive the result
    # ans = inv_vel_kinematics_client(q1,q2,q3,ee_velocity[0],ee_velocity[1],ee_velocity[2],ee_velocity[3],ee_velocity[4],ee_velocity[5])
    # joint_vel = ans.q_dot
    # print ("\nRequesting joint velocities...\n")
    # print ("Joint velocities [q1_dot,q2_dot,q3_dot]:\n{}".format(joint_vel))
