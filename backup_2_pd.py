#!/usr/bin/env python3
import sys
# ROS python API
import rospy

from sensor_msgs.msg import Imu
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Vector3
import tf.transformations as transformations
from tf.transformations import *
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt
from std_msgs.msg import Float32

from gazebo_msgs.msg import ModelStates

import numpy as np
from tf.transformations import *

from msg_check.msg import PlotDataMsg 


# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('/uav0/mavros/cmd/takeoff' and '/uav1/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('/uav0/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 13)

            takeoffService1 = rospy.ServiceProxy('/uav1/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService1(altitude = 13)

        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s"%e)

    def setArm(self):
        rospy.wait_for_service('/uav0/mavros/cmd/arming' and '/uav1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)

            armService1 = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService1(True)

        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode' and '/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')

            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='STABILIZED')

        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode'  and '/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')

            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='OFFBOARD')

        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode'  and '/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')

            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='ALTCTL')

        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode'  and '/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')

            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode'  and '/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')

            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='AUTO.LAND')

        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
           


class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        # set the flag to use position setpoints and yaw angle
        self.yaw_angle = Float32()
        self.yaw_angle.data = 0.0

        self.imu = Imu()
        
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 3.0
        self.sp.pose.position.z = self.ALT_SP

        self.local_pos = PoseStamped()
        self.local_vel = TwistStamped()

        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.desVel = np.zeros(3)
        self.desAcc = np.zeros(3)

        self.errInt = np.zeros(3)
        self.att_cmd = PoseStamped()
        self.thrust_cmd = Thrust()
        
# all data has been collected on this value
        self.Kpos_ = np.array([4, 4, 12])
        self.Kvel_ = np.array([3, 3, 3])
        self.Kint_ = np.array([0.2, 0.2, 3.0])

        self.norm_thrust_const = 0.056
        self.max_th = 36.0
        self.max_throttle = 0.96

        self.gravity = np.array([0, 0, 9.8])
        self.pre_time = rospy.get_time()

        self.armed = False

        self.command = AttitudeTarget()
        self.collective_thrust = 0.0

        # Publishers
        self.att_pub = rospy.Publisher('/uav0/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub = rospy.Publisher('/uav0/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.body_rate = rospy.Publisher('/uav0/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        self.att_pub1 = rospy.Publisher('/uav1/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub1 = rospy.Publisher('/uav1/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.body_rate1 = rospy.Publisher('/uav1/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        self.data_pub = rospy.Publisher('/data_out', PlotDataMsg, queue_size=10)

        self.data_out = PlotDataMsg()


    def base_link_pos(self, msg):
        idx = msg.name.index('iris0')
        iris_pose = msg.pose[idx]
        iris_twist = msg.twist[idx]
        self.local_pos.pose.position.x = iris_pose.position.x
        self.local_pos.pose.position.y = iris_pose.position.y
        self.local_pos.pose.position.z = iris_pose.position.z

        self.local_pos.pose.orientation.x = iris_pose.orientation.x
        self.local_pos.pose.orientation.y = iris_pose.orientation.y
        self.local_pos.pose.orientation.z = iris_pose.orientation.z
        self.local_pos.pose.orientation.w = iris_pose.orientation.w

        self.local_vel.twist.linear.x = iris_twist.linear.x
        self.local_vel.twist.linear.y = iris_twist.linear.y
        self.local_vel.twist.linear.z = iris_twist.linear.z

        self.local_vel.twist.angular.x = iris_twist.angular.x
        self.local_vel.twist.angular.y = iris_twist.angular.y
        self.local_vel.twist.angular.z = iris_twist.angular.z

    def posCb(self, msg):
        self.local_pos.pose.position.x = msg.pose.position.x
        self.local_pos.pose.position.y = msg.pose.position.y
        self.local_pos.pose.position.z = msg.pose.position.z
        self.local_pos.pose.orientation.x = msg.pose.orientation.x
        self.local_pos.pose.orientation.y = msg.pose.orientation.y
        self.local_pos.pose.orientation.z = msg.pose.orientation.z
        self.local_pos.pose.orientation.w = msg.pose.orientation.w

        print()

    def velCb(self, msg):
        self.local_vel.twist.linear.x = msg.twist.linear.x
        self.local_vel.twist.linear.y = msg.twist.linear.y
        self.local_vel.twist.linear.z = msg.twist.linear.z

        self.local_vel.twist.angular.x = msg.twist.angular.x
        self.local_vel.twist.angular.y = msg.twist.angular.y
        self.local_vel.twist.angular.z = msg.twist.angular.z


    def multiDoFCb(self, msg):
        pt = msg.points[0]
        self.sp.pose.position.x = pt.transforms[0].translation.x
        self.sp.pose.position.y = pt.transforms[0].translation.y
        self.sp.pose.position.z = pt.transforms[0].translation.z
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        self.desAcc = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])


    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.pose.position.x = self.local_pos.pose.position.x
        self.sp.pose.position.y = self.local_pos.pose.position.y
        self.sp.pose.position.z = self.local_pos.pose.position.z

    def accCB(self,msg):
        self.imu.orientation.w = msg.orientation.w
        self.imu.orientation.x = msg.orientation.x
        self.imu.orientation.y = msg.orientation.y
        self.imu.orientation.z = msg.orientation.z

        self.imu.angular_velocity.x = msg.angular_velocity.x
        self.imu.angular_velocity.y = msg.angular_velocity.y
        self.imu.angular_velocity.z = msg.angular_velocity.z

        self.imu.linear_acceleration.x = msg.linear_acceleration.x
        self.imu.linear_acceleration.y = msg.linear_acceleration.y
        self.imu.linear_acceleration.z = msg.linear_acceleration.z


    def newPoseCB(self, msg):
        if(self.sp.pose.position != msg.pose.position):
            print("New pose received")
        self.sp.pose.position.x = msg.pose.position.x        
        self.sp.pose.position.y = msg.pose.position.y
        self.sp.pose.position.z = msg.pose.position.z
   
        self.sp.pose.orientation.x = msg.pose.orientation.x
        self.sp.pose.orientation.y = msg.pose.orientation.y
        self.sp.pose.orientation.z = msg.pose.orientation.z
        self.sp.pose.orientation.w = msg.pose.orientation.w

    # def newPoseCB1(self, msg):
    #     if(self.sp.pose.position != msg.pose.position):
    #         print("New pose received")
    #     self.sp.pose.position.x = msg.pose.position.x        
    #     self.sp.pose.position.y = msg.pose.position.y
    #     self.sp.pose.position.z = msg.pose.position.z
   
    #     self.sp.pose.orientation.x = msg.pose.orientation.x
    #     self.sp.pose.orientation.y = msg.pose.orientation.y
    #     self.sp.pose.orientation.z = msg.pose.orientation.z
    #     self.sp.pose.orientation.w = msg.pose.orientation.w

    def yawAngle(self,msg):
        self.yaw_angle.data = msg.data
        self.yaw_angle.data = np.deg2rad(self.yaw_angle.data)

    def vector2Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z])

    def vector3Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z , vector.w])

    def array2Vector3(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]
        vector.z = array[2]

    def array2Vector4(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]
        vector.z = array[2]
        vector.w = array[3]


    def a_des(self):
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        if dt > 0.03:
            dt = 0.03

        curPos = self.vector2Arrays(self.local_pos.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position)
        curVel = self.vector2Arrays(self.local_vel.twist.linear)
        curAcc = self.vector2Arrays(self.imu.linear_acceleration) - self.gravity

        errPos = curPos - desPos
        errVel = curVel - self.desVel
        errAcc = curAcc - self.desAcc


#  DATA COLLECTION PURPOSE ONLY
        self.data_out.x_curr = curPos[0]
        self.data_out.y_curr = curPos[1]
        self.data_out.z_curr = curPos[2]
        self.data_out.x_dot_curr = curVel[0]
        self.data_out.y_dot_curr = curVel[1]
        self.data_out.z_dot_curr = curVel[2]
        self.data_out.x_ddot_curr = curAcc[0]
        self.data_out.y_ddot_curr = curAcc[1]
        self.data_out.z_ddot_curr = curAcc[2]
        self.data_out.x_des = desPos[0]
        self.data_out.y_des = desPos[1]
        self.data_out.z_des = desPos[2]
        self.data_out.x_dot_des = self.desVel[0]
        self.data_out.y_dot_des = self.desVel[1]
        self.data_out.z_dot_des = self.desVel[2]
        self.data_out.x_ddot_des = self.desAcc[0]
        self.data_out.y_ddot_des = self.desAcc[1]
        self.data_out.z_ddot_des = self.desAcc[2]
        self.data_out.x_err = errPos[0]
        self.data_out.y_err = errPos[1]
        self.data_out.z_err = errPos[2]
        self.data_out.x_dot_err = errVel[0]
        self.data_out.y_dot_err = errVel[1]
        self.data_out.z_dot_err = errVel[2]
        self.data_out.x_ddot_err = errAcc[0]
        self.data_out.y_ddot_err = errAcc[1]
        self.data_out.z_ddot_err = errAcc[2]

        if self.armed:
            self.errInt += errPos*dt

        des_a = -(self.Kpos_*errPos + self.Kvel_*errVel + self.Kint_*self.errInt) + self.gravity

        # print(errPos)
        # print(des_a)
        print('--------------')

        if np.linalg.norm(des_a) > self.max_th:
            des_a = (self.max_th/np.linalg.norm(des_a))*des_a

        return des_a 


    def geo_con_(self):
        des_a = self.a_des()

        pose = transformations.quaternion_matrix(  
                np.array([self.local_pos.pose.orientation.x, 
                             self.local_pos.pose.orientation.y, 
                             self.local_pos.pose.orientation.z, 
                             self.local_pos.pose.orientation.w]))  #4*4 matrix
        pose_temp1 = np.delete(pose, -1, axis=1)
        rot_curr = np.delete(pose_temp1, -1, axis=0)   #3*3 current rotation matrix
        zb_curr = rot_curr[:,2]
        thrust = self.norm_thrust_const * des_a.dot(zb_curr)
        self.collective_thrust = np.maximum(0.0, np.minimum(thrust, self.max_throttle))

        self.data_out.tau_x = des_a[0]
        self.data_out.tau_y = des_a[1]
        self.data_out.tau_z = des_a[2]
        self.data_out.thrust = des_a.dot(zb_curr)
        self.data_out.collective_thrust = thrust
        # ----------------------------------------------- # 

        # Current Euler orientation and 
        orientation_q = self.local_pos.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll_curr, pitch_curr, yaw_curr) = euler_from_quaternion (orientation_list)

        #---------------------------------------------#
        rot_des = self.acc2quat(des_a, self.yaw_angle.data)   #desired yaw = 0 Intially
        rot_44 = np.vstack((np.hstack((rot_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))

        quat_des = quaternion_from_matrix(rot_44)
        orientation_list = [quat_des[0], quat_des[1], quat_des[2], quat_des[3]]
        (roll_des, pitch_des, yaw_des) = euler_from_quaternion (orientation_list)

        # -------------------------------------------------#

        # DATA COLLECTION PUROSE ONLY
        angle_error_matrix = 0.5* (np.dot(np.transpose(rot_des), rot_curr) -
                                    np.dot(np.transpose(rot_curr), rot_des) ) #skew matrix
        roll_x_err = -angle_error_matrix[1,2]
        pitch_y_err = angle_error_matrix[0,2]   
        yaw_z_err = -angle_error_matrix[0,1]

        self.data_out.roll_dot_curr = self.local_vel.twist.angular.x
        self.data_out.pitch_dot_curr = self.local_vel.twist.angular.y
        self.data_out.yaw_dot_curr = self.local_vel.twist.angular.z
        self.data_out.roll_err = roll_x_err
        self.data_out.pitch_err = pitch_y_err
        self.data_out.yaw_err = yaw_z_err
        self.data_out.roll_curr = roll_curr
        self.data_out.pitch_curr = pitch_curr
        self.data_out.yaw_curr = yaw_curr
        self.data_out.roll_des = roll_des
        self.data_out.pitch_des = pitch_des
        self.data_out.yaw_des = yaw_des


    # -------------------------------
        # Control MODE 1
        now = rospy.Time.now()
        self.att_cmd.header.stamp = now
        self.thrust_cmd.header.stamp = now
        self.data_out.header.stamp = now
        self.att_cmd.pose.orientation.x = quat_des[0]
        self.att_cmd.pose.orientation.y = quat_des[1]
        self.att_cmd.pose.orientation.z = quat_des[2]
        self.att_cmd.pose.orientation.w = quat_des[3]
        self.thrust_cmd.thrust = self.collective_thrust


    def acc2quat(self,des_a, des_yaw):
        proj_xb_des = np.array([np.cos(des_yaw), np.sin(des_yaw), 0.0])
        if np.linalg.norm(des_a) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = des_a / np.linalg.norm(des_a)
        yb_des = np.cross(zb_des, proj_xb_des) / np.linalg.norm(np.cross(zb_des, proj_xb_des))
        xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([xb_des, yb_des, zb_des]))
        return rotmat

    def pub_att(self):
        self.geo_con_()
        self.thrust_pub.publish(self.thrust_cmd)
        self.att_pub.publish(self.att_cmd)
        self.data_pub.publish(self.data_out)

        self.thrust_pub1.publish(self.thrust_cmd)
        self.att_pub1.publish(self.att_cmd)
        # self.data_pub1.publish(self.data_out)


# Main function
def main(argv):
   
    rospy.init_node('setpoint_node', anonymous=True)

    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    rospy.Subscriber('/gazebo/model_states', ModelStates, cnt.base_link_pos)
    rospy.Subscriber('/uav0/mavros/state', State, cnt.stateCb)

    # rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    # rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)
    rospy.Subscriber('/uav0/mavros/imu/data', Imu, cnt.accCB)

    rospy.Subscriber('command/trajectory', Mdjt, cnt.multiDoFCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    rospy.Subscriber('yaw_in_deg',Float32,cnt.yawAngle)

    sp_pub = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    sp_pub1 = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    print("ARMING")
    print("done")
    while not cnt.state.armed:
        modes.setArm()
        cnt.armed = True
        rate.sleep()

    cnt.armed = True
    k=0

    

    while k<20:
        sp_pub.publish(cnt.sp)
        sp_pub1.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    # ROS main loop
    while not rospy.is_shutdown():

#--------------------------------------------
        cnt.pub_att()
        rate.sleep()
#--------------------------------------------  

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass

