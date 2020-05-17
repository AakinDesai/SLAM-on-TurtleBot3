#!/usr/bin/env python 
import rospy
import math
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose,Twist
from slam.msg import landmark
from tf.transformations import euler_from_quaternion, quaternion_from_euler

robot = "turtlebot3_burger"
obstacle = "landmark"
land_publish = rospy.Publisher('landmark_data',landmark, queue_size=0)


def processmodel(st,lv,av):
   
    sd = np.zeros(9)
    sd[0] = np.cos(st[2])*lv
    sd[1] = np.sin(st[2])*lv
    sd[2] = av
    st = st + (0.01*sd)
    return st 

def observationmodel(st):
    
    om = np.zeros(6)
    for i in range(3) :
	om[2*i] = math.hypot(st[3 + 2*i]-st[0],st[4 + 2*i]-st[1])
        om[2*i + 1] = math.atan2(st[4 + 2*i]-st[1],st[3 + 2*i]-st[0])
    return om
    
def timeupdate(st,cp,lv,av):

    global Q
    A = np.identity(9)
    A[0][2] = 0.01 * -np.sin(st[2])*lv
    A[0][3] = 0.01 * np.cos(st[2])*lv
    st = processmodel(st,lv,av)
    cp = np.matmul(np.matmul(A,cp),np.transpose(A)) + Q
    return st,cp

def measurementupdate(cp,st,ob):
	
    global R
    H = np.zeros((6,9))
    for i in range(3):
        H[2*i][0] = (-st[3 + 2*i]+st[0])/(math.hypot(st[3 + 2*i]-st[0],st[4 + 2*i]-st[1]))
        H[2*i][1] = (-st[4 + 2*i]+st[1])/(math.hypot(st[3 + 2*i]-st[0],st[4 + 2*i]-st[1]))
        H[2*i][3 + 2*i] = -H[2*i][0]
        H[2*i][4 + 2*i] = -H[2*i][1]
        H[2*i + 1][0] = (st[4 + 2*i]-st[1]) / ((math.hypot(st[3 + 2*i]-st[0],st[4 + 2*i]-st[1]))**2)
        H[2*i + 1][1] = (-st[3 + 2*i]+st[0]) / ((math.hypot(st[3 + 2*i]-st[0],st[4 + 2*i]-st[1]))**2)
        H[2*i + 1][3 + 2*i] = -H[2*i + 1][0]
        H[2*i + 1][4 + 2*i] = -H[2*i + 1][1]
   
    print(H)
    dum = np.matmul(np.matmul(H,cp),np.transpose(H)) + R
    dum = np.linalg.inv(dum) 
    K = np.matmul(np.matmul(cp,np.transpose(H)),dum)
    hx = observationmodel(st)
    st = st + np.matmul(K,ob-hx)
    I = np.identity(9)
    dum = I - np.matmul(K,H)
    cp = np.matmul(dum,cp)
    return st,cp

def callback(model):

    real = np.zeros(9)
    tb = model.name.index(robot)
    tb_pose = model.pose[tb]
    tb_twist = model.twist[tb]
    
    real[0] = tb_pose.position.x
    real[1] = tb_pose.position.y
    u1=tb_twist.linear.x
    u2=tb_twist.angular.z

    quaternion = (
    tb_pose.orientation.x,
    tb_pose.orientation.y,
    tb_pose.orientation.z,
    tb_pose.orientation.w)

    euler = euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    real[2] = yaw
 
    global state,P
    i = 3

    for k in range(len(model.name)):
        if obstacle in model.name[k]:

            state[i] = model.pose[k].position.x
            real[i] = model.pose[k].position.x
            i = i+1
	    state[i] = model.pose[k].position.y
	    real[i] = model.pose[k].position.y
            i = i+1
	
    state , P = timeupdate(state,P,u1,u2)
    observation = observationmodel(real)
    state , P = measurementupdate(P,state,observation)
    
    pub = landmark()
    pub.s = state
    
    land_publish.publish(pub)
    return 

state = np.zeros(9)
Q = np.zeros((9,9))
Q[0,0] = 0.1
Q[1,1] = 0.1
Q[2,2] = 0.1
P = np.identity(9)
R = 0* np.identity(6)
rospy.init_node('fake_data')
rospy.Subscriber('/gazebo/model_states',ModelStates,callback)
rospy.spin()




