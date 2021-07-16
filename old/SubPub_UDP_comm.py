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
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler                     # Modules needed for data processing

import types
from urdf_parser_py.urdf import URDF

#################################################################################################
#  Parameters for UDP communication with HRI: 

HRI_IP                = '127.0.0.1'                                                             # HRI IP
unity_calib_port      = 30012                                                                   # Ports
unity_flag_port       = 30011                                                                   #
logitech_gamepad_port = 30014                                                                   #

tags  = ['unity_calib_port','unity_flag_port', 'logitech_gamepad_port']                         # Socket tags (names)
ports = [ unity_calib_port , unity_flag_port ,  logitech_gamepad_port ]                         # Port list

#################################################################################################
#  Robot parameters:

#x_0   = 0.0                                                                                     # Robot position and orientation at spawn
#y_0   = 0.0                                                                                     #
#z_0   = 1.0                                                                                     #
#yaw_0 = 0.0                                                                                     #


class SubPub:
    SimFlag = None

    def __init__(self):
        self.placeholder                  = None
        self.message                      = None
        self.inputs                       = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
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
        self.uav_point_pos                = None
        self.uav_quat_or                  = None
        self.uav_lin_vel                  = None
        self.uav_ang_vel                  = None
        self.uav_speed                    = None
        self.pub_vel_ref                  = None                                                # Publishers
        self.pub_traj                     = None                                                #
        self.uav_roll                     = None
        self.uav_pitch                    = None
        self.uav_yaw                      = None
        self.message_timestamp            = None
        self.JointStatesData              = None
        self.debug_traj                   = None
        self.traj_no                      = None


    def CallbackLinkStates(self, data):
        idx = data.name.index('uav::base_link') 
        self.uav_point_pos = data.pose[idx].position
        self.uav_quat_or   = data.pose[idx].orientation
        self.uav_lin_vel   = data.twist[idx].linear
        self.uav_ang_vel   = data.twist[idx].angular
        """
        print type(math)
        print "uav_point_pos"
        print self.uav_point_pos
        print "uav_quat_or"
        print self.uav_quat_or
        print "uav_lin_vel"
        print self.uav_lin_vel
        print "uav_ang_vel"
        print self.uav_ang_vel
        """

    def RefreshDataForUDP(self):
        self.message_timestamp = rospy.get_rostime()        
        self.message = [self.inputs[0],self.inputs[1],self.inputs[2],self.inputs[3],self.inputs[4], self.inputs[5], self.inputs[6], self.inputs[7],self.inputs[8], self.inputs[9], self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, self.uav_lin_vel.x, self.uav_lin_vel.y, self.uav_lin_vel.z, self.uav_speed, self.uav_quat_or.x, self.uav_quat_or.y, self.uav_quat_or.z, self.uav_quat_or.w, self.uav_roll, self.uav_pitch, self.uav_yaw, self.uav_ang_vel.x, self.uav_ang_vel.y, self.uav_ang_vel.z, self.man_jnt_data_to_msg[0], self.man_jnt_data_to_msg[1], self.man_jnt_data_to_msg[2], self.man_jnt_data_to_msg[3], self.man_jnt_data_to_msg[4], self.man_jnt_data_to_msg[5], self.message_timestamp, self.state, self.maneuver, self.maneuver_duration, self.maneuver_max_amplitude, self.instance, self.loop_counter]
        self.msgs[tags.index('unity_calib_port')] = self.message
        self.msgs[tags.index('logitech_gamepad_port')] = self.inputs

    def CallbackClock(self, data):
        self.secs = data.secs
    
    def CallbackJoy(self, data):
        self.JoyData = data

    def CallbackJT(self, data):
        self.debug_traj = data
        print("Checked")

    def CallbackJointState(self, data):
        self.JointStatesData = data

    def CallbackExTraj(self, data):
        self.traj_no = data.data

    """
    def GetJointStates(self):
        CallbackKin = Kinematics()
        print "test0"

        while True:
            rospy.Subscriber('/uav/joint_states', JointState, self.CallbackJointState)
            print "test1"
            if self.JointStatesData is None:
                continue

            elif self.JointStatesData.header.frame_id != 'base_link':
                self.man_joint_qs   = self.JointStatesData.position
                self.man_joint_vels = self.JointStatesData.velocity
                if len(self.man_joint_qs) < 5 or len(self.man_joint_vels) < 5:
                    print "len error"
                    #print self.man_joint_qs
                    #print self.man_joint_qs
                    #print self.JointStatesData
                    continue
                else:
                    self.man_jnt_data_to_msg = CallbackKin.CLBCKVelSolve(self.man_joint_qs, self.man_joint_vels)
                    break
    """

    def Subs(self):
        CallbackKin = Kinematics()

       #rospy.Subscriber('/clock', Clock , self.CallbackClock)
       #rospy.Subscriber('/joy', Joy, self.CallbackJoy)
       #rospy.Subscriber('/uav/joint_trajectory', JointTrajectory , self.CallbackJT)


        while True:

            rospy.Subscriber('/gazebo/link_states', LinkStates , self.CallbackLinkStates)
            rospy.Subscriber('/uav/executing_trajectory', Int32, self.CallbackExTraj)
            rospy.Subscriber('/uav/joint_states', JointState, self.CallbackJointState)

            print("fetching joint state data")
            if not(self.JointStatesData is None or self.JointStatesData.header.frame_id == 'base_link'):
               
                self.man_joint_qs   = list(self.JointStatesData.position)
                self.man_joint_vels = list(self.JointStatesData.velocity)

                if not(len(self.man_joint_qs) < 5 or len(self.man_joint_vels) < 5):
                    #print("self.man_joint_qs")
                    #print(self.man_joint_qs)
                    #print("self.man_joint_vels")
                    #print(self.man_joint_vels)
                    self.man_jnt_data_to_msg = CallbackKin.CLBCKVelSolve(self.man_joint_qs, self.man_joint_vels)
                    break

            rospy.sleep(0.015)

        self.uav_speed = math.sqrt( abs(self.uav_lin_vel.x)**2 + abs(self.uav_lin_vel.y)**2 + abs(self.uav_lin_vel.z)**2 )
        (self.uav_roll, self.uav_pitch, self.uav_yaw) = euler_from_quaternion([self.uav_quat_or.x, self.uav_quat_or.y, self.uav_quat_or.z, self.uav_quat_or.w])


    def PubInit(self):
       #self.pub_vel_ref = rospy.Publisher('/uav/vel_ref', Vector3, queue_size=10)
        self.pub_traj = rospy.Publisher('/uav/joint_trajectory', JointTrajectory, queue_size=10)


    def AcqRoutine(self):
        K  = Kinematics()
        TP = TrajectoryPlanning()
        self.msgs[tags.index('unity_flag_port')] = 'a'

        self.Subs()
        self.RefreshDataForUDP()

        joint_list    = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        j_0           = [ self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]

        #init_traj = GenTraj.SetInitPose([-math.pi/2.0, math.pi/2.0, 0.0, 0.0, 0.0], 4.0,j_0)
        #chain = K.GenChain()


        q_start = [0.0, 0.0,math.pi/2, -0.436343, 0.892081, -0.455771]

        for seq in [99]:
            if seq == 99:
                traj = TP.InitPosition(seq, j_0, q_start )
            #elif seq == 98:
            #    traj = TP.InitPosition(seq, j_0, q_target)

            self.pub_traj.publish(traj)

            while self.traj_no == 0:
                rospy.Subscriber('/uav/executing_trajectory', Int32, self.CallbackExTraj)
                print("waiting for init position trajectory start")

            while self.traj_no != 0:
                rospy.Subscriber('/uav/executing_trajectory', Int32, self.CallbackExTraj)
                print("going to init position")
                

        #rospy.sleep(1.0)
        self.Subs()
        self.RefreshDataForUDP()
        j_0 = [self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, 0.0, 0.0, self.uav_yaw, self.man_joint_qs[0], self.man_joint_qs[1], self.man_joint_qs[2], self.man_joint_qs[3], self.man_joint_qs[4]]
        print j_0

        for seq in [1, 2, 3, 4, 5, 6, 7]:
            if seq == 5 or seq == 6 or seq == 7:
                self.maneuver = 4
            else:
                self.maneuver = seq

            if seq == 1:
                traj = TP.GenSinXYZRoutineTraj(seq, ["x"], [0.5], 6.0, j_0)

            elif seq == 2:
                traj = TP.GenSinXYZRoutineTraj(seq, ["y"], [0.5], 6.0, j_0)

            elif seq == 3:
                traj = TP.GenSinXYZRoutineTraj(seq, ["z"], [0.5], 6.0, j_0)

            elif seq == 4:
                traj = TP.GenSinXYZRoutineTraj(seq,["yaw"],[0.5], 6.0, j_0)

            elif seq == 5:
                j_s = [self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                #               x     y      z      r     p    y
                target_point = [0.0, -0.2797, 0.19749, 1.5707963267948966, 0.0, 0.0]
                traj = TP.GoToPoint(seq = seq, NoRot = True, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True)
                last_traj = traj

            elif seq == 6:
                traj = TP.FlipTraj(seq = seq, traj = last_traj)

            elif seq == 7:
                j_s = [self.uav_point_pos.x, self.uav_point_pos.y, self.uav_point_pos.z, 0.0 ,0.0 , q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]
                #               x     y      z      r     p    y
                target_point = [0.0, -0.295, 0.19749, 1.5707963267948966, 0.0, 0.0]
                traj = TP.GoToPoint(seq = seq, NoRot = True, time = 4.0, j_0 = j_s, target_pos = target_point, Force5DOF = True)
                last_traj = traj

            self.maneuver_duration = float(len(traj.points))/100.0 
            #print self.maneuver_duration   
            self.pub_traj.publish(traj)
            while self.traj_no == 0:
                print("waiting for trajectory start")

            while self.traj_no != 0:
                self.RefreshDataForUDP()
                self.UDP_send()
                print "Recording trajectory"



        #(ok, target_point) = K.Forward([math.pi*0.1, 0.0, 0.0, 0.0, 0.0])
        #(ok, res_pos, frame) = K.Inverse(target_point, [0.0, 0.0, 0.0, 0.0, 0.0])


        #traj_0 = GenTraj.GenFowardTraj(res_pos,4.0,  j_0)
        #self.pub_traj.publish(traj_0)
        #kin = GT.Kinematics()
        

        self.msgs[tags.index('unity_flag_port')] = 'q'   


    def setup_sockets(self):

        for i in range(len(ports)):
            self.socket_away_list[i]=socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        
    def UDP_send(self, msg_list):

        for i in range(len(self.socket_away_list)):
            if msg_list[i] is not None:

                if i == tags.index('unity_flag_port'):
                    data = pack('c', *msg_list[i])

                else:
                    data = pack('%sf' % len(msg_list[i]), *msg_list[i])
                    
                self.socket_away_list[i].sendto(data, (HRI_IP, ports[i]))

    
    def Run(self):
        self.setup_sockets()
        self.PubInit()
        rospy.init_node('SubPub_UDP_comm', anonymous=True)

        self.msgs = [None for x in range(len(tags))]
        self.msgs[tags.index('unity_flag_port')] = 'q'

        self.AcqRoutine()


if __name__ == '__main__':
    try:
        SP = SubPub()
        SP.Run()
        #rospy.spin()

    except rospy.ROSInterruptException: 
        print("rospy.ROSInterruptException")
        pass    

