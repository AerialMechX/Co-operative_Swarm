#!/usr/bin/env python
import random

import sys
import rospy
import numpy as np
from std_msgs.msg import Float64,Float64MultiArray
from sensor_msgs.msg import JointState


class Trajectory:

    def __init__(self):
        # Create a ROS publisher
        self.traj_pub = rospy.Publisher("/traj_pos", Float64MultiArray, queue_size=10)
        self.arm_error_pub_data = rospy.Publisher("/arm_error_data", Float64MultiArray, queue_size=10)

        self.count = 0
        self.pre_time = rospy.get_time()    

        self.curent_m2loc_init = 0.0
        self.activateM2, self.activateM1 = 0, 0
        self.countM2,self.countM1 = 0, 0
        self.lastm1_cout, self.lastm2_cout = 0, 0

        # self.step = 3.0
        self.step1 = 6.0
        self.step2 = 8.0

        # self.step1Pos_offset, self.step2Pos_offset = 0.8, 0.8  

        # self.des_Vel = 0.1
        
        self.intial_count1 = 0
        self.intial_count2 = 0

        self.m1_torque, self.m2_torque = 0.0, 0.0

        self.joint1Pos, self.joint1Vel = 0.0, 0.0
        self.joint2Pos, self.joint2Vel = 0.0, 0.0

        self.joint1Pos_rel, self.joint2Pos_rel = 0, -90


        self.errPos_m1, self.errVel_m1 = 0.0, 0.0
        self.errPos_m2, self.errVel_m2 = 0.0, 0.0

        self.desPos_m1, self.desVel_m1, self.desAcc_m1 = (0), 0.0, 0.0  #conf1
        self.desPos_m2, self.desVel_m2, self.desAcc_m2 = (-90), 0.0, 0.0

        self.desPos_m2_loc = -90.0
        self.desPos_m1_loc = 0.0


    def conf(self, msg):
        self.conf = msg.data
        # print(self.conf)
        # if self.conf == 1:
        #     self.desPos_m1, self.desPos_m2 = 0, -90
        # if self.conf == 2:
        #     self.desPos_m1, self.desPos_m2 = 0, 90
        # if self.conf == 3:
        #     self.desPos_m1, self.desPos_m2 = -45, 50
        # if self.conf == 4:
        #     self.desPos_m1, self.desPos_m2 = 50, -45


        if self.conf == 1:
            self.desPos_m1, self.desPos_m2 = 0, 90 #(id_1, id_2)
        # if self.conf == 2:
        #     self.desPos_m1, self.desPos_m2 = 0, 90
        # if self.conf == 3:
        #     self.desPos_m1, self.desPos_m2 = -45, 50
        # if self.conf == 4:
        #     self.desPos_m1, self.desPos_m2 = 50, -45


        if np.absolute(self.joint2Pos_rel - self.desPos_m2) >= self.step2:
            self.activateM2, self.countM2 = 1, 1
        
        if np.absolute(self.joint1Pos_rel - self.desPos_m1) >= self.step1:
            self.activateM1, self.countM1 = 1, 1

        # print(self.desPos_m1, self.desPos_m2)

    def joint_states(self, msg):
        ind_m1 = msg.name.index('id_1')
        ind_m2 = msg.name.index('id_2')

        self.joint1Pos = (msg.position[ind_m1])
        self.joint1Vel = msg.velocity[ind_m1]
        # self.joint1Effort = msg.effort[ind_m1]

        self.joint2Pos = (msg.position[ind_m2])
        self.joint2Vel = msg.velocity[ind_m2]
        # self.joint2Effort = msg.effort[ind_m2]

    def pi_2_pi(self, angle):
        while(angle > np.pi):
            angle = angle - 2.0 * np.pi
        while(angle < -np.pi):
            angle = angle + 2.0 * np.pi
        return angle

    def desired_pose(self):
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        # if dt > 0.04:
            # dt = 0.04

        if self.count == 0:
            self.intial_count1 = self.joint1Pos
            self.intial_count2 = self.joint2Pos - np.pi/2
            self.count = 1
            print("Count = 0")

        self.joint1Pos_rel = np.rad2deg(self.pi_2_pi(-self.joint1Pos + self.intial_count1))
        self.errPos_m1 = self.joint1Pos_rel - self.desPos_m1
        self.desVel_m1 = (0.0) 

        if self.activateM1 == 1:
            # print("here")
            if self.countM1 == 1:
                # print("here")
                self.curent_m1loc_init = self.joint1Pos_rel
                self.countM1 = 0

            self.curent_m1loc_init = self.curent_m1loc_init - np.sign(self.errPos_m1)*self.step1
            self.desPos_m1_loc = self.curent_m1loc_init
            if self.lastm1_cout == 0:
                self.last_desPos_m1_loc = self.desPos_m1_loc
                self.lastm1_cout = 1

            self.desVel_m1 = -(self.desPos_m1_loc - self.last_desPos_m1_loc)/(dt*300)
            self.last_desPos_m1_loc = self.desPos_m1_loc

            if np.absolute(self.curent_m1loc_init - self.desPos_m1) <= self.step1:
                # print("here")
                self.activateM1 = 0
                self.countM1 = 0
                self.desVel_m1 = (0.0) 

        self.M1_pos_err = np.rad2deg(self.pi_2_pi(self.joint1Pos)) + self.desPos_m1_loc
        self.M1_vel_err = self.joint1Vel - self.desVel_m1




        self.joint2Pos_rel = np.rad2deg(self.pi_2_pi(-self.joint2Pos + self.intial_count2))
        # print(self.joint1Pos_rel, self.joint2Pos_rel)
        self.errPos_m2 = self.joint2Pos_rel - self.desPos_m2
        self.desVel_m2 = (0.0) 

        if self.activateM2 == 1:
            # print("here")
            if self.countM2 == 1:
                # print("here")
                self.curent_m2loc_init = self.joint2Pos_rel
                self.countM2 = 0

            self.curent_m2loc_init = self.curent_m2loc_init - np.sign(self.errPos_m2)*self.step2
            self.desPos_m2_loc = self.curent_m2loc_init

            if self.lastm2_cout == 0:
                self.last_desPos_m2_loc = self.desPos_m2_loc
                self.lastm2_cout = 1

            self.desVel_m2 = -(self.desPos_m2_loc - self.last_desPos_m2_loc)/(dt*300)
            self.last_desPos_m2_loc = self.desPos_m2_loc

            if np.absolute(self.curent_m2loc_init - self.desPos_m2) <= self.step2:
                # print("here")
                self.activateM2 = 0
                self.countM2 = 0
                self.desVel_m2 = (0.0) 

        # print("M2 :", self.desPos_m2_loc, 
        #             np.rad2deg(self.pi_2_pi(self.joint2Pos)) + self.desPos_m2_loc,
        #             self.joint2Vel, self.desVel_m2, self.joint2Vel - self.desVel_m2)
        self.M2_pos_err = np.rad2deg(self.pi_2_pi(self.joint2Pos)) + self.desPos_m2_loc
        self.M2_vel_err = self.joint2Vel - self.desVel_m2
        # print("-----------------")

    def publish_trajectory(self):
        self.desired_pose()
        pub_data = Float64MultiArray()
        pub_data.data = np.array([self.desPos_m1_loc,self.desVel_m1,self.desPos_m2_loc,self.desVel_m2])
        self.traj_pub.publish(pub_data)
        # print(self.desPos_m1_loc, self.desPos_m2_loc)

        arm_error_pub_data = Float64MultiArray()
        arm_error_pub_data.data = np.array([-self.M1_pos_err, -self.M1_vel_err,
                                            self.M2_pos_err,self.M2_vel_err])
        # print(-self.M1_pos_err, -self.M1_vel_err, 
        #         self.M2_pos_err, self.M2_vel_err)
        self.arm_error_pub_data.publish(arm_error_pub_data)





def main(argv):
    rospy.init_node("trajectory_node")
    traj = Trajectory()

    rospy.Subscriber('/motor_conf', Float64, traj.conf)

    # Subscribe the raw marker data
    motors_states = rospy.wait_for_message('/joint_states', JointState)
    traj.joint_states(motors_states)    
    rospy.Subscriber('/joint_states', JointState, traj.joint_states)

    # Create a rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # traj.joint_states()
        traj.publish_trajectory()
        rate.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass