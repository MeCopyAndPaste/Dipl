#!/usr/bin/env python
import rospy                                                                                    # All of the big, "default" modules
import math                                                                                     #
import numpy as np                                                                              #
import socket                                                                                   #
import matplotlib.pyplot as plt                                                                 #
import time                                                                                     #
import rosbag                                                                                   # Bagging module
#import BagHandle  as BH                                                                         # Module for needed for rosbag - under construction
from struct import *                                                                            # Needed for packing UDP data
#import PyKDL
from GenerateTrajectories import Kinematics, TrajectoryPlanning 

from std_msgs.msg import String                                                                 # ROS messages needed for subs/pubs
from rosgraph_msgs.msg import Clock                                                             #
from gazebo_msgs.msg import LinkStates                                                          #
from sensor_msgs.msg import Joy                                                                 #
from geometry_msgs.msg import Pose                                                              #
from geometry_msgs.msg import Vector3                                                           #
from trajectory_msgs.msg import JointTrajectoryPoint                                            #
from trajectory_msgs.msg import JointTrajectory                                                 #
from std_msgs.msg import Header, Int32
from sensor_msgs.msg import JointState, Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler                     # Modules needed for data processing

import types
from urdf_parser_py.urdf import URDF

#################################################################################################
#  Parameters for UDP communication with HRI: 

HRI_IP                = '127.0.0.1'                                                             # HRI IP
unity_calib_port      = 30012                                                                   # Ports
unity_flag_port       = 30011                                                                   #
logitech_gamepad_port = 30014                                                                   #
imus_port             = 29001

tags  = ['unity_calib_port','unity_flag_port', 'logitech_gamepad_port', 'imus_port']                         # Socket tags (names)
ports = [ unity_calib_port , unity_flag_port ,  logitech_gamepad_port ,  imus_port ]                         # Port list

types_in_imus = 'qqccccccccdddddddddd'

input_device  = 'imus'
#################################################################################################
#  Robot parameters:

#x_0   = 0.0                                                                                     # Robot position and orientation at spawn
#y_0   = 0.0                                                                                     #
#z_0   = 1.0                                                                                     #
#yaw_0 = 0.0                                                                                     #


class SubPub:
    SimFlag = None

    def __init__(self):
        self.placeholder                  = 0
        self.message                      = 0
        self.inputs                       = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.state                        = 0                                                   
        self.maneuver                     = 0
        self.maneuver_duration            = 0                                                   
        self.maneuver_max_amplitude       = 0
        self.instance                     = 0
        self.loop_counter                 = 0                                                   
        self.secs                         = None                                                
#       self.sock                         = None                                                
        self.socket_away_list             = [None for x in range(len(tags))]                    
        self.msgs                         = None
        self.man_jnt_data_to_msg          = [0, 0, 0, 0, 0]                                                
        self.man_joint_vels               = [0, 0, 0, 0, 0]
        self.man_joint_qs                 = [0, 0, 0, 0, 0]                                  
        self.JoyData                      = None
        self.bag                          = None
        self.uav_point_pos                = 0
        self.uav_quat_or                  = 0
        self.uav_lin_vel                  = 0
        self.uav_ang_vel                  = 0
        self.uav_speed                    = 0
        self.pub_vel_ref                  = None                                                # Publishers
        self.pub_traj                     = None                                                #
        self.uav_roll                     = 0
        self.uav_pitch                    = 0
        self.uav_yaw                      = 0
        self.message_timestamp            = 0
        self.JointStatesData              = 0
        self.debug_traj                   = 0
        self.traj_no                      = 0
        self.imus_message                 = 0
        self.active_topics                = []
        self.imu0                         = [None for x in range(len(types_in_imus))]
        self.imu1                         = [None for x in range(len(types_in_imus))]
        self.imu2                         = [None for x in range(len(types_in_imus))]
        self.imu3                         = [None for x in range(len(types_in_imus))]
        self.imu4                         = [None for x in range(len(types_in_imus))]
        self.imu5                         = [None for x in range(len(types_in_imus))]
        self.imu6                         = [None for x in range(len(types_in_imus))]
        self.imu7                         = [None for x in range(len(types_in_imus))]

    def Subs(self):
        rospy.Subscriber('/gazebo/link_states', LinkStates , self.CallbackLinkStates)
        self.active_topics.append(False)
        rospy.Subscriber('/uav/executing_trajectory', Int32, self.CallbackExTraj)
        self.active_topics.append(False)
        rospy.Subscriber('/uav/joint_states', JointState, self.CallbackJointState)
        self.active_topics.append(False)
        if input_device == 'imus':
            rospy.Subscriber('/imu0', Imu, self.CallbackImu2)
            self.active_topics.append(False)
            rospy.Subscriber('/imu1', Imu, self.CallbackImu3)
            self.active_topics.append(False)
            rospy.Subscriber('/imu2', Imu, self.CallbackImu1)
            self.active_topics.append(False)
            rospy.Subscriber('/imu3', Imu, self.CallbackImu4)
            self.active_topics.append(False)
            rospy.Subscriber('/imu4', Imu, self.CallbackImu5)
            self.active_topics.append(False)
            rospy.Subscriber('/imu5', Imu, self.CallbackImu0)
            self.active_topics.append(False)
            rospy.Subscriber('/imu6', Imu, self.CallbackImu6)
            self.active_topics.append(False)
            rospy.Subscriber('/imu7', Imu, self.CallbackImu7)
            self.active_topics.append(False)
        if input_device == 'joy':
            rospy.Subscriber('/joy', Joy, self.CallbackJointState)
            self.active_topics.append(False)

    def WaitForTopicsToActivate(self):
        Continue = False


        first = False
        
        while not(Continue):

            if not(first):
                print("Waiting for all topics to become active.")
                first = True                

            Continue = True
            for i in range(len(self.active_topics)):
                if self.active_topics[i] == False:
                    Continue = False
                


        print("All topics are now active. Continuing.")

    def setup_sockets(self):

        for i in range(len(ports)):
            self.socket_away_list[i]=socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
 
    def RefreshDataForUDP(self):
        ros_time = rospy.get_rostime()
        self.message_timestamp = ros_time.secs + ros_time.nsecs/1000000000

        imus = [self.imu0, self.imu1, self.imu2, self.imu3, self.imu4, self.imu5, self.imu7 ]
        #imus = [self.imu0, self.imu1, self.imu2, self.imu3, self.imu4, self.imu5, self.imu6, self.imu7 ]         
        self.imus_message = []
        for i in range(len(imus)):
            imus[i][0] = ros_time.secs
            imus[i][1] = ros_time.nsecs
            for j in range(len(imus[i])):
                self.imus_message.append(imus[i][j])

        self.message = [self.inputs[0],self.inputs[1],self.inputs[2],self.inputs[3],self.inputs[4], self.inputs[5], self.inputs[6], self.inputs[7],self.inputs[8], self.inputs[9], self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, self.uav_lin_vel.x, self.uav_lin_vel.y, self.uav_lin_vel.z, self.uav_speed, self.uav_quat_or.x, self.uav_quat_or.y, self.uav_quat_or.z, self.uav_quat_or.w, self.uav_roll, self.uav_pitch, self.uav_yaw, self.uav_ang_vel.x, self.uav_ang_vel.y, self.uav_ang_vel.z, self.man_jnt_data_to_msg[0], self.man_jnt_data_to_msg[1], self.man_jnt_data_to_msg[2], self.man_jnt_data_to_msg[3], self.man_jnt_data_to_msg[4], self.man_jnt_data_to_msg[5], self.message_timestamp, self.state, self.maneuver, self.maneuver_duration, self.maneuver_max_amplitude, self.instance, self.loop_counter]
        self.msgs[tags.index('unity_calib_port')] = self.message
        self.msgs[tags.index('logitech_gamepad_port')] = self.inputs
        self.msgs[tags.index('imus_port')] = self.imus_message
   
    def UDP_send(self, msg_list, s):

        for i in range(len(self.socket_away_list)):
            if msg_list[i] is not None:

                if i == tags.index('unity_flag_port'):
                    data = pack('c', *msg_list[i])
                    self.socket_away_list[i].sendto(data, (HRI_IP, ports[i]))                   

                elif i == tags.index('imus_port') and input_device == 'imus':
                    l = len(self.imus_message)/len(types_in_imus)
                    stri = types_in_imus*l
                    data = pack(stri, *msg_list[i])
                    self.socket_away_list[i].sendto(data, (HRI_IP, ports[i]))

                elif input_device == 'joy':
                    data = pack('%sf' % len(msg_list[i]), *msg_list[i])
                    self.socket_away_list[i].sendto(data, (HRI_IP, ports[i]))

                elif i == tags.index('unity_calib_port'):
                    data = pack('%sf' % len(msg_list[i]), *msg_list[i])
                    self.socket_away_list[i].sendto(data, (HRI_IP, ports[i]))
                    #print("\nmsg:")
                    #print(msg_list[i])

    def CallbackLinkStates(self, data):
        idx = data.name.index('uav::base_link') 
        self.uav_point_pos = data.pose[idx].position
        self.uav_quat_or   = data.pose[idx].orientation
        self.uav_lin_vel   = data.twist[idx].linear
        self.uav_ang_vel   = data.twist[idx].angular

        self.uav_speed = math.sqrt( abs(self.uav_lin_vel.x)**2 + abs(self.uav_lin_vel.y)**2 + abs(self.uav_lin_vel.z)**2 )
        (self.uav_roll, self.uav_pitch, self.uav_yaw) = euler_from_quaternion([self.uav_quat_or.x, self.uav_quat_or.y, self.uav_quat_or.z, self.uav_quat_or.w])

        self.active_topics[0] = True

    def CallbackJoy(self, data):
        self.JoyData = data

        self.active_topics[3] = True

    def CallbackJointState(self, data):
        CallbackKin = Kinematics()
        self.JointStatesData = data
        if not(self.JointStatesData is None or self.JointStatesData.header.frame_id == 'base_link'):
               
            self.man_joint_qs   = list(self.JointStatesData.position)
            self.man_joint_vels = list(self.JointStatesData.velocity)

            if not(len(self.man_joint_qs) < 5 or len(self.man_joint_vels) < 5):
                self.man_jnt_data_to_msg = CallbackKin.CLBCKVelSolve(self.man_joint_qs, self.man_joint_vels)
                self.active_topics[2] = True
            
    def CallbackExTraj(self, data):
        self.traj_no = data.data
        self.active_topics[1] = True

    def CallbackImu0(self, data):
        (r, p ,y) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.imu0  = [data.header.stamp.secs, data.header.stamp.nsecs, '0','0','b','4','2','2','1','8', 0.0, 0.0, 0.0, r, p, y, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.active_topics[3] = True

    def CallbackImu1(self, data):
        (r, p ,y) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.imu1  = [data.header.stamp.secs, data.header.stamp.nsecs, '0','0','b','4','2','1','f','6', 0.0, 0.0, 0.0, r, p, y, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.active_topics[4] = True
        
    def CallbackImu2(self, data):
        (r, p ,y) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.imu2  = [data.header.stamp.secs, data.header.stamp.nsecs, '0','0','b','4','2','2','1','6', 0.0, 0.0, 0.0, r, p, y, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.active_topics[5] = True
        
    def CallbackImu3(self, data):
        (r, p ,y) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.imu3  = [data.header.stamp.secs, data.header.stamp.nsecs, '0','0','b','4','2','2','1','7', 0.0, 0.0, 0.0, r, p, y, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.active_topics[6] = True
        
    def CallbackImu4(self, data):
        (r, p ,y) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.imu4  = [data.header.stamp.secs, data.header.stamp.nsecs, '0','0','b','4','3','c','3','c', 0.0, 0.0, 0.0, r, p, y, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.active_topics[7] = True
        
    def CallbackImu5(self, data):
        (r, p ,y) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.imu5  = [data.header.stamp.secs, data.header.stamp.nsecs, '0','0','b','4','3','c','4','1', 0.0, 0.0, 0.0, r, p, y, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.active_topics[8] = True
        
    def CallbackImu6(self, data):
        (r, p ,y) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.imu6  = [data.header.stamp.secs, data.header.stamp.nsecs, '0','0','b','4','2','2','1','7', 0.0, 0.0, 0.0, r, p, y, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.active_topics[9] = True
        
    def CallbackImu7(self, data):
        (r, p ,y) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.imu7  = [data.header.stamp.secs, data.header.stamp.nsecs, '0','0','b','4','3','c','5','d', 0.0, 0.0, 0.0, r, p, y, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.active_topics[10] = True
        
    def PubInit(self):
       #self.pub_vel_ref = rospy.Publisher('/uav/vel_ref', Vector3, queue_size=10)
        self.pub_traj = rospy.Publisher('/uav/joint_trajectory', JointTrajectory, queue_size=10)

    def AcqRoutine(self, device, do_user_ready_routine):
        K  = Kinematics()
        TP = TrajectoryPlanning()
        self.msgs[tags.index('unity_flag_port')] = 'a'
        rospy.sleep(1.0)

        self.RefreshDataForUDP()

        joint_list    = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        j_0           = [ self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]

        default_x = self.uav_point_pos.x
        default_y = self.uav_point_pos.y
        default_z = self.uav_point_pos.z

        q_start = [0.0, math.pi/6.0, math.pi/3.0, -0.436343, 0.892081, -0.455771]

        print("Going to init position.")
        if do_user_ready_routine:
            rospy.sleep(8.0)
            for seq in [99]:
                if seq == 99:
                    traj = TP.InitPosition(seq, j_0, q_start )

                self.pub_traj.publish(traj)

                while self.traj_no == 0:
                    #print("waiting for init position trajectory start")
                    pass
                while self.traj_no != 0:
                    pass
            print("Arrived into init position.")
            print("\n")
            print("\n")
            print("")
            print("Get ready")
            print("\n")
            print("\n")
            rospy.sleep(2.0)

            self.RefreshDataForUDP()
            j_0 = [self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
        
            print("Acquisition starts in:")
            for x in [5, 4, 3, 2, 1]:
                print(x)
                rospy.sleep(1.0) 

        else:
            rospy.sleep(2.0)
            self.RefreshDataForUDP()
            j_0 = [self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
        


        print("ACQUISITION STARTED!")
        print('\a')
        print("\n")
        print("\n")

        last_target =  [0, -0.3265+0.05, 0.1811, 1.57, 0.0, 0.0]
        target_point = [0, -0.3265, 0.1811, 1.57, 0.0, 0.0]  #   x y  z   r p y

        #for seq in [1, 2, 3, 4]:
        #for seq in [5, 6, 7, 8, 9, 10, 11, 12, 13, 14]:
        for seq in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]:#

            print("Executing trajectory no:")
            print(seq)

            if seq == 1:
                traj = TP.GenSinRoutineTraj(seq, ["x"], [0.5], 6.0, j_0)
                self.maneuver               = 0
                self.maneuver_duration      = 6.0
                self.maneuver_max_amplitude = 0.5

            elif seq == 2:
                traj = TP.GenSinRoutineTraj(seq, ["x"], [-0.5], 6.0, j_0)
                self.maneuver               = 1
                self.maneuver_duration      = 6.0
                self.maneuver_max_amplitude = 0.5

            elif seq == 3:
                traj = TP.GenSinRoutineTraj(seq, ["y"], [0.5], 6.0, j_0)
                self.maneuver               = 10
                self.maneuver_duration      = 6.0
                self.maneuver_max_amplitude = 0.5 

            elif seq == 4:
                traj = TP.GenSinRoutineTraj(seq, ["y"], [-0.5], 6.0, j_0)
                self.maneuver               = 9
                self.maneuver_duration      = 6.0
                self.maneuver_max_amplitude = 0.5

            elif seq == 5:
                traj = TP.GenSinRoutineTraj(seq, ["z"], [0.5], 6.0, j_0)
                self.maneuver               = 4
                self.maneuver_duration      = 6.0
                self.maneuver_max_amplitude = 0.5

            elif seq == 6:
                traj = TP.GenSinRoutineTraj(seq,["yaw"],[0.5], 6.0, j_0)
                self.maneuver               = 3
                self.maneuver_duration      = 6.0
                self.maneuver_max_amplitude = 0.5

            elif seq == 7:
                traj = TP.GenSinRoutineTraj(seq,["yaw"],[-0.5], 6.0, j_0)
                self.maneuver               = 2
                self.maneuver_duration      = 6.0
                self.maneuver_max_amplitude = 0.5

            elif seq == 8:
                q_start = [0.0, math.pi/6.0, math.pi/3.0, -0.436343, 0.892081, -0.455771]
                j_s = [default_x, default_y, default_z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                target_point = [0, -0.3265, 0.1811, 1.57, 0.0, 0.0]  #   x y  z   r p y
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = False, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True)
                last_traj = traj
                self.maneuver               = 31
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq == 9:
                traj = TP.FlipTraj(seq = seq, traj = last_traj)
                self.maneuver               = 30
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq == 10:
                target_point = [0, -0.3265, 0.1811, 1.57, 0.0, 0.0] #   x y  z   r p y
                q_start = [0.0, math.pi/6.0, math.pi/3.0, -0.436343, 0.892081, -0.455771]
                j_s = [default_x, default_y, default_z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = False, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True)
                last_traj = traj
                self.maneuver               = 31
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq == 11:
                target_point = [-0.11, -0.3265, 0.1811, 1.57, 0.0, 0.0] #   x y  z   r p y
                j_s = [default_x, default_y, default_z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = True, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True, step = 0.0003)
                last_traj = traj
                self.maneuver               = 32
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq == 12:
                target_point = [ 0.11, -0.3265, 0.1811, 1.57, 0.0, 0.0] #   x y  z   r p y
                j_s = [default_x, default_y, default_z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = True, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True, step = 0.0003)
                last_traj = traj           
                self.maneuver               = 33
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq ==13:
                target_point = [ 0.0, -0.3265, 0.1811, 1.57, 0.0, 0.0] #   x y  z   r p y
                j_s = [default_x, default_y, default_z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = True, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True, step = 0.0003)
                last_traj = traj
                self.maneuver               = 32
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq ==14:
                target_point = [ 0.0, -0.3515, 0.1811, 1.57, 0.0, 0.0] #   x y  z   r p y
                j_s = [self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = False, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True)
                last_traj = traj
                self.maneuver               = 30
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq ==15:
                target_point = [ 0.0, -0.3515, 0.195, 1.5707963267948966, 0.0, 0.0] #   x y  z   r p y
                j_s = [default_x, default_y, default_z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = False, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True)
                last_traj = traj
                self.maneuver               = 34
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq ==16:
                target_point = [ 0.0, -0.3515, 0.159, 1.5707963267948966, 0.0, 0.0] #   x y  z   r p y
                j_s = [default_x, default_y, default_z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = False, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True)
                last_traj = traj
                self.maneuver               = 35
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq ==17:
                target_point = [ 0.0, -0.3515, 0.1811, 1.57, 0.0, 0.0] #   x y  z   r p y
                j_s = [default_x, default_y, default_z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                (q_start, traj) = TP.GoToPoint(seq = seq, NoRot = False, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True)
                last_traj = traj
                self.maneuver               = 34
                self.maneuver_duration      = float(len(traj.points))/100.0
                self.maneuver_max_amplitude = math.sqrt((target_point[0]-last_target[0])**2 + (target_point[1]-last_target[1])**2+ (target_point[2]-last_target[2])**2)/self.maneuver_duration 

            elif seq ==18:
                j_0 = [default_x, default_y, default_z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
                traj = TP.GenSinRoutineTraj(seq, ["man_yaw"],        [-1.0*(math.pi/3.0)*(math.pi/5.0)], 5.0, j_0)
                self.maneuver               = 36
                self.maneuver_duration      = 5.0
                self.maneuver_max_amplitude = -1.0*(math.pi/3.0)*(math.pi/5.0) 

            elif seq ==19:
                j_0 = [default_x, default_y, default_z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
                traj = TP.GenSinRoutineTraj(seq, ["man_yaw"],        [ 1.0*(math.pi/3.0)*(math.pi/5.0)], 5.0, j_0)
                self.maneuver               = 37
                self.maneuver_duration      = 5.0
                self.maneuver_max_amplitude = 1.0*(math.pi/3.0)*(math.pi/5.0) 

            elif seq ==20:
                j_0 = [default_x, default_y, default_z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
                traj = TP.GenSinRoutineTraj(seq, ["man_pitch"],      [ 1.0*(math.pi/5.0)*(math.pi/18.0)], 5.0, j_0)
                self.maneuver               = 39
                self.maneuver_duration      = 5.0
                self.maneuver_max_amplitude = 1.0*(math.pi/5.0)*(math.pi/18.0) 

            elif seq ==21:
                j_0 = [default_x, default_y, default_z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
                traj = TP.GenSinRoutineTraj(seq, ["man_pitch"],      [-1.0*(math.pi/5.0)*(math.pi/3.0)], 5.0, j_0)
                self.maneuver               = 38
                self.maneuver_duration      = 5.0
                self.maneuver_max_amplitude = 1.0*(math.pi/5.0)*(math.pi/3.0) 

            elif seq ==22:
                j_0 = [default_x, default_y, default_z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
                traj = TP.GenSinRoutineTraj(seq, ["man_pitch_base"], [ 1.0*(math.pi/5.0)*(math.pi/18.0)], 5.0, j_0)
                self.maneuver               = 41
                self.maneuver_duration      = 5.0
                self.maneuver_max_amplitude = 1.0*(math.pi/5.0)*(math.pi/18.0) 

            elif seq ==23:
                j_0 = [default_x, default_y, default_z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
                traj = TP.GenSinRoutineTraj(seq, ["man_pitch_base"], [-1.0*(math.pi/5.0)*(math.pi/3.0)], 5.0, j_0)
                self.maneuver               = 40

                self.maneuver_duration      = 5.0
                self.maneuver_max_amplitude = -1.0*(math.pi/5.0)*(math.pi/3.0) 

            last_target = target_point
            #self.maneuver_duration = float(len(traj.points))/100.0 
            #print self.maneuver_duration   
            self.pub_traj.publish(traj)
            while self.traj_no == 0:
                #print("waiting for trajectory to start")
                self.RefreshDataForUDP()

            while self.traj_no != 0:
                self.RefreshDataForUDP()
                self.UDP_send(self.msgs, 'imus')
                #print [self.message[10:15], self.message[10:15]]
                #print "Recording trajectory"



        #(ok, target_point) = K.Forward([math.pi*0.1, 0.0, 0.0, 0.0, 0.0])
        #(ok, res_pos, frame) = K.Inverse(target_point, [0.0, 0.0, 0.0, 0.0, 0.0])


        #traj_0 = GenTraj.GenFowardTraj(res_pos,4.0,  j_0)
        #self.pub_traj.publish(traj_0)
        #kin = GT.Kinematics()
        

        self.msgs[tags.index('unity_flag_port')] = 'q'   
        self.RefreshDataForUDP()
        self.UDP_send(self.msgs, 'imus')

    def Run(self):
        self.setup_sockets()
        self.PubInit()
        rospy.init_node('SubPub_UDP_comm', anonymous=True)

        self.Subs()
        #self.WaitForTopicsToActivate()

        self.msgs = [None for x in range(len(tags))]
        self.msgs[tags.index('unity_flag_port')] = 'q'

        self.AcqRoutine(input_device, True)


if __name__ == '__main__':
    try:
        SP = SubPub()
        SP.Run()
        #rospy.spin()

    except rospy.ROSInterruptException: 
        print("rospy.ROSInterruptException")
        pass    

