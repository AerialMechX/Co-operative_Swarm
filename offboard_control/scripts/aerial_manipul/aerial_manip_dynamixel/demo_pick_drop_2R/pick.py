#!/usr/bin/env python
import sys
# ROS python API
import rospy

from std_msgs.msg import *
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Accel, Vector3
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt
from msg_check.msg import PlotDataMsg
# from scipy import linalg as la

import dynamixel_sdk
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from tf import *


import numpy as np
import tf
import RPi.GPIO as GPIO
import time
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class dynamixtest:


    # def shutdown(self):
    #     print('shutdown')
    #     dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
    #     print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    #     self.write_to_motor(0)

    def __init__(self,id):

        self.PRESENT_POSITION        = 0

        self.OPERATING_MODE          = 2 #Position mode

        # Control table address
        self.ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        self.ADDR_GOAL_POSITION      = 116
        self.ADDR_PRESENT_POSITION   = 132
        self.ADDR_OPERATING_MODE     = 11

        # Protocol version
        self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

        # Default setting
        # DXL_ID                      = 1                 # Dynamixel ID : 1
        self.DXL_ID                  = id                
        print(self.DXL_ID)

        self.BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
        # DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

        self.DXL_MAXIMUM_POSITION_VALUE  = 10000*(1024/90)            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        self.init_motor_connection()


        # portHandler = PortHandler(DEVICENAME)
        # packetHandler = PacketHandler(PROTOCOL_VERSION)


        # self.M1_POS = 0
        # self.M1_VEL = 0

        # self.ADDR_TORQUE_ENABLE      = 64
        # self.ADDR_GOAL_POSITION      = 116
        # self.ADDR_PRESENT_POSITION   = 132

        # self.GOAL_POSITION           = 0
        # self.PRESENT_POSITION        = 0
        # self.GOAL_POSITION_ANGLE = 0

        # self.ADDR_GOAL_CURRENT       = 102
        # self.ADDR_PRESENT_CURRENT    = 126
        # self.GOAL_CURRENT            = 0 #-1193 to +1193
        # self.PRESENT_CURRENT         = 0

        # self.ADDR_GOAL_VELOCITY      = 104
        # self.ADDR_PRESENT_VELOCITY   = 128
        # self.GOAL_VELOCITY           = 0
        # self.PRESENT_VELOCITY        = 0

        # self.ADDR_OPERATING_MODE     = 11
        # self.OPERATING_MODE          = 0 #Current Mode 
        #self.OPERATING_MODE          = 3 #Position mode  

        # self.PROTOCOL_VERSION        = 2.0
        # self.DXL_ID                  = id                
        # self.BAUDRATE                = 57600            
        # self.DEVICENAME              = '/dev/ttyUSB0' 
        # self.TORQUE_ENABLE           = 1


        # if self.DXL_ID == 1:    
        #     self.kp = 0.2
        #     self.ki = 0.0
        #     self.kd = 0.6
        # if self.DXL_ID == 2:    
        #     self.kp = 0
        #     self.ki = 0
        #     self.kd = 0
        # self.errInt = 0
        # self.pre_time = rospy.get_time()

        # self.error = 0
        # self.previous_error=0

        # self.pub = rospy.Publisher('error'+str(self.DXL_ID), Float32, queue_size=10)
        # self.pub2 = rospy.Publisher('zero'+str(self.DXL_ID), Int32, queue_size=10)

        # self.sub2 = rospy.Subscriber("dynamixel"+str(self.DXL_ID)+"_control", Float64MultiArray, self.data_set1)

        # self.start_val = 0
        # self.init_motor_connection()
        # self.tick_ratio = (1024/90) 



    # def data_set1(self,data): # callback for dynamixel
    #     self.M1_POS = data.data[0]
    #     self.M1_VEL = data.data[1]
    #     self.GOAL_POSITION_ANGLE = self.M1_POS
    #     self.GOAL_VELOCITY = self.M1_VEL



    def init_motor_connection(self):  #do not change 
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            getch()
            quit()

        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            getch()
            quit()


        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
        # print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        # print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            getch()
            quit()
        else:
            print("DYNAMIXEL has been successfully connected")


    def get_motor_data(self):
        self.PRESENT_POSITION, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        # self.PRESENT_VELOCITY, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)


    def set_goal_pos_callback(self, position):
        # print("Set Goal Position of ID %s = %s" % (data.id, data.position*(1024/90)))
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, position *(1024/90) )


def main(argv):
   
    print("All Good")

    dynamixel_obj0 = dynamixtest(0)
    time.sleep(0.5)
    dynamixel_obj1 = dynamixtest(1)
    time.sleep(0.5)
    dynamixel_obj2 = dynamixtest(2)
    time.sleep(0.5)
    dynamixel_obj3 = dynamixtest(3)
    time.sleep(0.5)
    print("enabled")

    # BE CAREFUL OF THE MOTOR IDs 

    dynamixel_obj0.set_goal_pos_callback(140) #motor 1 first
    dynamixel_obj2.set_goal_pos_callback(130) #link 2 motor
    dynamixel_obj1.set_goal_pos_callback(180) #link 3
    time.sleep(1)
    dynamixel_obj3.set_goal_pos_callback(235) #gripper
    print("enabled2")

    # dynamixel_obj1.set_goal_pos_callback(160) #motor 1 first
    # dynamixel_obj2.set_goal_pos_callback(175) #gripper actuator motor
    # dynamixel_obj3.set_goal_pos_callback(160) #link 2 motor
    # dynamixel_obj4.set_goal_pos_callback(151)


if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
