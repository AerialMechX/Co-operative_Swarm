#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float32,Float64MultiArray,Int32
import time

class Trajectory:

	def __init__(self):
		self.sub1 = rospy.Subscriber("dynamixel1_cont", Float32, self.cb_mot1)
		self.sub2 = rospy.Subscriber("dynamixel2_cont", Float32, self.cb_mot2)

		self.pub1 = rospy.Publisher('dynamixel1_control', Float64MultiArray, queue_size=10)
		self.pub2 = rospy.Publisher('dynamixel2_control', Float64MultiArray, queue_size=10)

		self.mot1_target = -90
		self.mot1_current = 0
		self.mot1_step = 5

		self.mot2_target = 0
		self.mot2_current = 0
		self.mot2_step = 5

		self.thre = 2

		self.t = 1
		self.k = 1

		#with k 0.5 max speed is achieved

	def cb_mot1(self,data):
		self.mot1_target = data.data

	def cb_mot2(self,data):
		self.mot2_target = data.data
	
	def publish_trajectory(self):
		if (self.mot1_target - self.mot1_current) > self.thre: 
			self.mot1_current = self.mot1_current + (self.mot1_step)

		elif (self.mot1_target - self.mot1_current) < -self.thre:
			self.mot1_current = self.mot1_current - (self.mot1_step)

		if (self.mot2_target - self.mot2_current) > self.thre: 
			self.mot2_current = self.mot2_current + (self.mot2_step)

		elif (self.mot2_target - self.mot2_current) < -self.thre:
			self.mot2_current = self.mot2_current - (self.mot2_step)

		pubs1 = Float64MultiArray()
		if (self.mot1_target - self.mot1_current) <= self.thre and (self.mot1_target - self.mot1_current) >= -self.thre: 
			pubs1.data = np.array([self.mot1_current,0])
		else:
			pubs1.data = np.array([self.mot1_current,self.mot1_step/self.t])

		pubs2 = Float64MultiArray()
		if (self.mot2_target - self.mot2_current) <= self.thre and (self.mot2_target - self.mot2_current) >= -self.thre:
			pubs2.data = np.array([self.mot2_current,0])
		else:
			pubs2.data = np.array([self.mot2_current,self.mot2_step/self.t])

		self.pub1.publish(pubs1)
		self.pub2.publish(pubs2)
		#print(self.mot1_step/self.t)

	def set_time(self,t):
		self.t = t

def main(argv):
    rospy.init_node("dynamix_trajectory_node")
    traj = Trajectory()

    rate = rospy.Rate(10)

    t_prev = time.time()

    while not rospy.is_shutdown():
    	t=time.time()
    	traj.set_time(t-t_prev)
        traj.publish_trajectory()
        rate.sleep()
        t_prev = t

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass