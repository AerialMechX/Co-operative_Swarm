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
import os, sys

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


    def shutdown(self):
        print('shutdown')
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        

    def __init__(self,id):

        self.PRESENT_POSITION        = 0

        self.OPERATING_MODE          = 0 # CURRENT/TORQUE MODE = 0; POSITION MODE = 3; PWM MODE = 16

        self.PWM_LIMIT               = 500 # RANGE: 0 ~ 885

        self.CURRENT_LIMIT           = 500 # RANGE: 0 ~ 1,193 ; UNIT : 2.69mA 

        # Control table address
        self.ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        self.ADDR_GOAL_POSITION      = 116
        self.ADDR_PRESENT_POSITION   = 132
        self.ADDR_OPERATING_MODE     = 11
        self.ADDR_GOAL_PWM           = 100
        self.ADDR_PWM_LIMIT          = 36
        self.ADDR_GOAL_CURRENT       = 102
        self.ADDR_CURRENT_LIMIT      = 38
        self.ADDR_PRESENT_CURRENT    = 126
        self.ADDR_PRESENT_PWM        = 124

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


        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("TORQUE_DISABLE", self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE))

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("OPERATING_MODE", self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE))        

        if self.OPERATING_MODE == 16: # PWM MODE
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PWM_LIMIT, self.PWM_LIMIT)
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print("PWM_LIMIT", self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PWM_LIMIT))
        elif self.OPERATING_MODE == 0: # CURRENT MODE
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_CURRENT_LIMIT, self.CURRENT_LIMIT)
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print("CURRENT_LIMIT", self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_CURRENT_LIMIT))


        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("TORQUE_ENABLE", self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE))

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

    def set_goal_pwm_callback(self, pwm):   ## pwm range: ~PWM_LIMIT to PWM_LIMIT
        # print("Set Goal Position of ID %s = %s" % (data.id, data.pwm))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm )
        print("set_goal_pwm_callback :", self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM))
        
    def set_goal_current_callback(self, current):   ## current range: ~CURRENT_LIMIT to CURRENT_LIMIT
        # print("Set Goal Position of ID %s = %s" % (data.id, data.pwm))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, current )
        print("set_goal_current_callback :", self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT))



def main(argv):
   
    print("All Good")

    
    dynamixel_obj1 = dynamixtest(1)
    time.sleep(0.5)

    print("enabled")

    # BE CAREFUL OF THE MOTOR IDs 

    # dynamixel_obj1.set_goal_pos_callback(100)

    # dynamixel_obj1.set_goal_pwm_callback(200)

    dynamixel_obj1.set_goal_current_callback(100)


    time.sleep(1)


    while not rospy.is_shutdown():
        dxl_present_current, dxl_comm_result, dxl_error = dynamixel_obj1.packetHandler.read2ByteTxRx(dynamixel_obj1.portHandler, dynamixel_obj1.DXL_ID, dynamixel_obj1.ADDR_PRESENT_CURRENT)
        print("Present current : ",dxl_present_current) # CURRENT
       
        # dxl_present_pwm, dxl_comm_result, dxl_error = dynamixel_obj1.packetHandler.read2ByteTxRx(dynamixel_obj1.portHandler, dynamixel_obj1.DXL_ID, dynamixel_obj1.ADDR_PRESENT_PWM) # PWM
        # print("Present pwm : ",dxl_present_pwm) # PWM

        # dxl_present_position, dxl_comm_result, dxl_error = dynamixel_obj1.packetHandler.read4ByteTxRx(dynamixel_obj1.portHandler, dynamixel_obj1.DXL_ID, dynamixel_obj1.ADDR_PRESENT_POSITION) # POSITION
        # print("Present position : ",dxl_present_position) # POSITION

        ESC_ASCII_VALUE = 27 # ESC ASCII VALUE
        if  getch() == chr(ESC_ASCII_VALUE):
            dynamixel_obj1.shutdown()
            break
        else:
            pass


if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass