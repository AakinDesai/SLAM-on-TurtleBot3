#!/usr/bin/env python 
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose,Twist
from slam.msg import landmark
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import roslib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


robot = "turtlebot3_burger"
obstacle = "landmarks"

land_publish = rospy.Publisher('landmark_data',landmark, queue_size=0)
od_publish = rospy.Publisher('odom1',Odometry, queue_size=0)
joint_publish =rospy.Publisher('joint_states1', JointState, queue_size=10)
marker_publish = rospy.Publisher('visualization_marker_array', MarkerArray,queue_size=10)


def processmodel(st,lv,av):
    
    global dt
    sd = np.zeros(9)
    sd[0] = np.cos(st[2])*lv
    sd[1] = np.sin(st[2])*lv
    sd[2] = av
    st = st + (dt*sd)
    return st 

def observationmodel(st):
    
    om = np.zeros(6)
    for i in range(3) :
	om[2*i] = math.hypot(st[3 + 2*i]-st[0],st[4 + 2*i]-st[1])
        om[2*i + 1] = math.atan2(st[4 + 2*i]-st[1],st[3 + 2*i]-st[0])
    return om
    
def timeupdate(st,cp,lv,av):

    global Q,dt
    A = np.identity(9)
    A[0][2] = dt * -np.sin(st[2])*lv
    A[0][3] = dt * np.cos(st[2])*lv
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
   
    dum = np.matmul(np.matmul(H,cp),np.transpose(H)) + R
    dum = np.linalg.inv(dum) 
    K = np.matmul(np.matmul(cp,np.transpose(H)),dum)
    hx = observationmodel(st)
    st = st + np.matmul(K,ob-hx)
    I = np.identity(9)
    dum = I - np.matmul(K,H)
    cp = np.matmul(dum,cp)
    return st,cp

def callback2(msg):
    
    global u1,u2
  
    u1 = msg.linear.x
    u2 = msg.angular.z
    
            
def callback(model):

    global u1,u2
    real = np.zeros(9)
    tb = model.name.index(robot)
    tb_pose = model.pose[tb]
    tb_twist = model.twist[tb]
    
    real[0] = tb_pose.position.x

    real[1] = tb_pose.position.y
    rospy.Subscriber("/cmd_vel", Twist, callback2)
    
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
 
    global state,P,wl,wr,last,dt,odox,odoy,odot
    i = 3
 
    markerArray = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.z = 0.0
    m = 0
    
    for k in range(len(model.name)):
        if obstacle in model.name[k]:

            state[i] = model.pose[k].position.x
            real[i] = model.pose[k].position.x
            marker.pose.position.x = model.pose[k].position.x
            i = i+1
	    state[i] = model.pose[k].position.y
	    real[i] = model.pose[k].position.y
            marker.pose.position.y = model.pose[k].position.y
            marker.id = m
            m = m + 1 
            i = i+1
            markerArray.markers.append(marker)
            marker_publish.publish(markerArray)


    now = rospy.get_rostime()
    dt = (now.nsecs - last.nsecs) * (10 ** -9)
    state , P = timeupdate(state,P,u1,u2)
    last =   rospy.get_rostime()
    observation = observationmodel(real)
    state , P = measurementupdate(P,state,observation)
    error = state-real

    vx = np.cos(odot)*u1
    vy = np.sin(odot)*u1
    wz = u2
    odox = odox + (dt*vx)
    odoy = odoy + (dt*vy)
    odot = odot + (dt*wz)
    oq = quaternion_from_euler(0,0,odot)
  
    pub = landmark()
    pub.s = error
    
    land_publish.publish(pub)
    
    odom_ = Odometry()
    odom_.header.frame_id = "odom"
    odom_.child_frame_id = "base_footprint"
    odom_.pose.pose.position.x = state[0]
    odom_.pose.pose.position.y = state[1]
    odom_.pose.pose.position.z = 0;

    odomq = quaternion_from_euler(0,0,state[2])
    odom_.pose.pose.orientation.x = odomq[0]
    odom_.pose.pose.orientation.y = odomq[1]
    odom_.pose.pose.orientation.z = odomq[2]
    odom_.pose.pose.orientation.w = odomq[3]

    odom_.twist.twist.linear.x  = u1;
    odom_.twist.twist.angular.z = u2;

    od_publish.publish(odom_)
	
    a = (u1*dt*2)/(0.033)
    b = (u2*dt*0.160)/(0.033)

    r = (a+b)/(2)
    l = (a-b)/(2)
    wl = wl + l
    wr = wr + r

    js = JointState()
    js.header.frame_id = "base_footprint"
    js.header.stamp = now;
    js.name = ['wheel_left_link' , 'wheel_right_l']
    js.position= [wl,wr]
    
    joint_publish.publish(js)

	
    odom_tf = geometry_msgs.msg.TransformStamped()
    odom_tf.header = odom_.header;
    odom_tf.child_frame_id = odom_.child_frame_id;
    odom_tf.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf.transform.rotation = odom_.pose.pose.orientation;
    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(odom_tf)
  

state = np.zeros(9)
odox = 0
odoy = 0
odot = 0
Q = np.zeros((9,9))
Q[0,0] = 0.1
Q[1,1] = 0.1
Q[2,2] = 0.1
P = np.identity(9)
R = 0.1 * np.identity(6)
wl = 0
wr = 0
u1 = 0
u2 = 0
dt = 0
rospy.init_node('fake')
last =   rospy.get_rostime()
rospy.Subscriber('/gazebo/model_states',ModelStates,callback)
rospy.spin()

