#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
import numpy as np
import struct
from fb5_torque_ctrl.msg import encoderData
from fb5_torque_ctrl.msg import PwmInput
from tf.transformations import euler_from_quaternion, quaternion_from_euler


current_time = 0
pos_x = 0
pos_y = 0
yaw = 0
iterations = 3
wheel_dist = 5
pwms = np.linspace(0,255,52)
bot_odom_topic = "/vicon/bot1/odom/"

# def Position(msg):
# 	global current_time,pos_x,pos_y,yaw
# 	current_time = msg.header.stamp.secs
# 	pos_x = pose.pose.postion.x
# 	pos_y = pose.pose.postion.y
# 	orientation_q = msg.pose.pose.orientation
# 	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
# 	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

rospy.init_node("autocalibration")
pub_PWM=rospy.Publisher('pwmCmd',PwmInput,queue_size=10)
# start_time = rospy.wait_for_message(bot_odom_topic, Odometry).header.stamp.secs
start_time = 0
print(start_time)
# rospy.Subscriber(bot_odom_topic,Odometry, Position)

for pwm in pwms:
	for run_id in range(iterations):
		data = np.zeros([0,4])
		while current_time-start_time < 10 :
			data = np.append(data, np.array([[current_time,pos_x,pos_y,yaw]]), axis=0)
			pwmInput.rightInput=pwm
			pwmInput.leftInput=pwm
			pub_PWM.publish(pwmInput)
			current_time += 1e-03
                        
		np.save("./run_"+str(run_id)+"pwm_"+str(pwm)+"_forward.npy",data)
		start_time = current_time

		while current_time-start_time < 10 :
			data = np.append(data, np.array([[current_time,pos_x,pos_y,yaw]]), axis=0)
			pwmInput.rightInput= -pwm
			pwmInput.leftInput= -pwm
			pub_PWM.publish(pwmInput)
            current_time += 1e-03
		np.save("./run_"+str(run_id)+"pwm_"+str(pwm)+"_backward.npy",data)
		start_time = current_time


plot_data = np.zeros([0,3])
for pwm in pwms:
	V_runs = []
	W_runs = []
	for run_id in range(iterations):
		data = np.load("./run_"+str(run_id)+"pwm_"+str(pwm))
		t = data[:,0]
		x_path = data[:,1]
		y_path = data[:,2]
		thetas = data[:,3]
		dt = np.ediff1d(t)
		dx = np.ediff1d(x_path)
		dy = np.ediff1d(y_path)
		dtheta = np.ediff1d(thetas)
		ds = np.sqrt(dx*dx+dy*dy)
		v = np.average(ds/dt)
		w = np.average(d_theta/dt)
		V_runs.append(v)
		W_runs.append(w)

	V = np.average(V_runs)
	W = np.average(W_runs)
	Vr = (V + W*wheel_dist)/2
	Vl = (V - W*wheel_dist)/2
	plot_data= np.append(plot_data, np.array([[Vl,Vr,pwm]]), axis=0)

np.save("pwm_vs_wheel_velocities.npy",plot_data)




