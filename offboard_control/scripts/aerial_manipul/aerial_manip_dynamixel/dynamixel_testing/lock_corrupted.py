#!/usr/bin/env python3
import os
import rospy
import dynamixel_sdk
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
import numpy as np
from std_msgs.msg import Float32,Int32
import time

class dynamixtest:

    def shutdown(self):
        print('shutdown')
        #self.write_to_motor(0)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        #self.write_to_motor(0)

    def __init__(self,id,OPERATING_MODE,ratio=1):
        rospy.init_node('dynatest')
        rospy.on_shutdown(self.shutdown)

        self.ADDR_TORQUE_ENABLE      = 64

        self.ADDR_GOAL_POSITION      = 116
        self.ADDR_PRESENT_POSITION   = 132

        self.PRESENT_POSITION        = 0
        self.GOAL_POSITION           = 0
        self.GOAL_POSITION_ANGLE     = 0
        self.ADDR_OPERATING_MODE =11

        self.OPERATING_MODE          = OPERATING_MODE #Position mode  

        self.PROTOCOL_VERSION        = 2.0

        self.DXL_ID                  = id                
        self.BAUDRATE                = 57600            
        self.DEVICENAME              = '/dev/ttyUSB0' 
        self.TORQUE_ENABLE           = 1

        self.ratio = 11.3777777778 * ratio

        self.sub = rospy.Subscriber("dynamixel"+str(self.DXL_ID)+"_control", Int32, self.data_set1)

        self.init_motor_connection()

    def data_set1(self,data):
        self.GOAL_POSITION_ANGLE = data.data
        self.write_to_motor(int(self.GOAL_POSITION_ANGLE * self.ratio))

    def init_motor_connection(self):
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            quit()

        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            quit()

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("TORQUE DISABLED")


        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))


        print("OPERATING_MODE ", self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE))


        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            quit()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            quit()
        else:
            print("DYNAMIXEL has been successfully connected")

    def write_to_motor(self,val):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, val)


def main(argv):
   
    print("All Good")
    
    obj2 = dynamixtest(id=1,OPERATING_MODE=3,ratio=1)

    obj2.write_to_motor(180)

    time.sleep(0.5)

    print("enabled")

    # BE CAREFUL OF THE MOTOR IDs 

    # dynamixel_obj1.set_goal_pos_callback(100)

    dynamixel_obj1.write_to_motor(50)

    # dynamixel_obj1.set_goal_current_callback(0)


if __name__ == '__main__':
    # obj1 = dynamixtest(id=0,OPERATING_MODE=3,ratio=1)
    # obj3 = dynamixtest(id=2,OPERATING_MODE=4,ratio=1079.73632812)
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
    # obj1.shutdown()
    obj2.shutdown()
    # obj3.shutdown()
