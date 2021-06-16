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
import GenerateTrajectories as GT

from std_msgs.msg import String                                                                 # ROS messages needed for subs/pubs
from rosgraph_msgs.msg import Clock                                                             #
from gazebo_msgs.msg import LinkStates                                                          #
from sensor_msgs.msg import Joy                                                                 #
from geometry_msgs.msg import Pose                                                              #
from geometry_msgs.msg import Vector3                                                           #
from trajectory_msgs.msg import JointTrajectoryPoint                                            #
from trajectory_msgs.msg import JointTrajectory                                                 #
from std_msgs.msg import Header                                                                 #

from tf.transformations import euler_from_quaternion, quaternion_from_euler                     # Modules needed for data processing


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

x_0   = 0.0                                                                                     # Robot position and orientation at spawn
y_0   = 0.0                                                                                     #
z_0   = 1.0                                                                                     #
yaw_0 = 0.0                                                                                     #


class SubPub:
    SimFlag = None

    def __init__(self):
        self.placeholder                  = None
        self.message                      = None
        self.dummy_inputs                 = [1, 1, 2, 3, 5, 8, 13, 0, 0, 0]
        self.dummy_state                  = 0                                                   
        self.dummy_maneuver               = 0
        self.dummy_maneuver_duration      = 0                                                   
        self.dummy_maneuver_max_amplitude = 0
        self.dummy_instance               = 0
        self.dummy_loop_counter           = 0                                                   
        self.secs                         = None                                                
#       self.sock                         = None                                                
        self.socket_away_list             = [None for x in range(len(tags))]                    
        self.msgs                         = None                                                
        self.man_joint_vels               = [0, 0, 0, 0, 0, 0]                                  
        self.JoyData                      = None
        self.bag                          = None

        self.pub_vel_ref                  = None                                                # Publishers
        self.pub_traj                     = None                                                #

        self.debug_traj  = None

    def CallbackLinkStates(self, data):
#       self.bag.write('uav_base_link',data)
        idx = data.name.index('uav::base_link') 
        point_pos = data.pose[idx].position
        quat_or   = data.pose[idx].orientation
        lin_vel   = data.twist[idx].linear
        ang_vel   = data.twist[idx].angular
        speed     = math.sqrt( abs(lin_vel.x)**2 + abs(lin_vel.y)**2 + abs(lin_vel.z)**2 )

        (roll, pitch, yaw) = euler_from_quaternion([quat_or.x, quat_or.y, quat_or.z, quat_or.w])

        self.message = [self.dummy_inputs[0],self.dummy_inputs[1],self.dummy_inputs[2],self.dummy_inputs[3],self.dummy_inputs[4], self.dummy_inputs[5], self.dummy_inputs[6], self.dummy_inputs[7],self.dummy_inputs[8], self.dummy_inputs[9], point_pos.x, point_pos.y, point_pos.z, lin_vel.x, lin_vel.y, lin_vel.z, speed, quat_or.x, quat_or.y, quat_or.z, quat_or.w, roll, pitch, yaw, ang_vel.x, ang_vel.y, ang_vel.z, self.man_joint_vels[0], self.man_joint_vels[1], self.man_joint_vels[2], self.man_joint_vels[3], self.man_joint_vels[4], self.man_joint_vels[5], rospy.get_time(), self.dummy_state, self.dummy_maneuver, self.dummy_maneuver_duration, self.dummy_maneuver_max_amplitude, self.dummy_instance, self.dummy_loop_counter]

#        print(self.message)
#        print(len(self.message))
#        print(type(self.message))

#        self.message = np.char.array([self.dummy_inputs[0],self.dummy_inputs[1],self.dummy_inputs[2],self.dummy_inputs[3],self.dummy_inputs[4], self.dummy_inputs[5], point_pos.x, point_pos.y, point_pos.z, lin_vel.x, lin_vel.y, lin_vel.z, speed, quat_or.x, quat_or.y, quat_or.z, quat_or.w, roll, pitch, yaw, rospy.get_time(), self.dummy_state, self.dummy_maneuver, self.dummy_maneuver_duration, self.dummy_maneuver_max_amplitude, self.dummy_instance, self.dummy_loop_counter])

#        np.char.array(['input1', 'input2', 'input3', 'input4', 'input5', 'input6', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'speed', 'quat_x', 'quat_y', 'quat_z','quat_w', 'roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate', 'timestamp', 'state', 'maneuver', 'maneuver duration', 'maneuver max amplitude', 'instance', 'loop counter'])
#        print(self.message)

#        self.msgs[tags.index('unity_flag_port')] = 'a'
        self.msgs[tags.index('unity_calib_port')] = self.message
        self.msgs[tags.index('logitech_gamepad_port')] = [1, 1, 2, 3, 5, 8, 13, 0, 0, 0]
        self.UDP_send(self.msgs)

#        print(self.msgs.size)
#        data = pack(fmt, self.message)



    def CallbackClock(self, data):
        self.secs = data.secs
    
    def CallbackJoy(self, data):
#        self.bag.write('joy',data)
        self.JoyData = data
    def CallbackJT(self, data):
        self.debug_traj = data
        print("Checked")

    def Subs(self):
        
       #rospy.Subscriber('/clock', Clock , self.CallbackClock)
       #rospy.Subscriber('/joy', Joy, self.CallbackJoy)
        rospy.Subscriber('/gazebo/link_states', LinkStates , self.CallbackLinkStates)
        rospy.Subscriber('/uav/joint_trajectory', JointTrajectory , self.CallbackJT)

    def PubInit(self):
       #self.pub_vel_ref = rospy.Publisher('/uav/vel_ref', Vector3, queue_size=10)
        self.pub_traj = rospy.Publisher('/uav/joint_trajectory', JointTrajectory, queue_size=10)


    def AcqRoutine(self):
        K       = GT.Kinematics()
        GenTraj = GT.TrajectoryPlanning()
        self.msgs[tags.index('unity_flag_port')] = 'a'

#                   x    y    z    r     p            y 
#        target_point = [0.5, 0.0, 0.2, 0.0, -math.pi/2.0, 0.0]

        man_refs    = [0.0, 0.0, 0.0, 0.0   , 0.0   , 0.0        , math.pi*0.25, math.pi*0.25, math.pi*0.25, math.pi*0.25, math.pi*0.25]
        man_refs2   = [1.0, 1.0, 1.0, 0.0   , 0.0   , math.pi/2.0, 0.0         , 0.0         , 0.0         , 0.0         , 0.0         ]

        joint_list    = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        j_0           = [ 0.0, 0.0, 1.0, 0.0, 0,0, 0.0, 0.0, 0.0, 0.0, 0,0, 0.0]
        init_traj = GenTraj.SetInitPose([-math.pi/2.0, math.pi/2.0, 0.0, 0.0, 0.0], 4.0,j_0)

        self.pub_traj.publish(init_traj)



#                                               A    T    Points/second
#        trajectory1 = GenTraj.GenSinXYZRoutineTraj(0.5,12.0, 100)
#        t = GenTraj.GenSinXYZRoutineTraj(0.5,12.0, 100)


        #man_refs      = [math.pi*0.0, math.pi*0.0, math.pi*0.25, math.pi*0.0, math.pi*0.0]
        #traj_man = GenTraj.GenJointsManRoutine(man_refs, 12.0, j_0)

        (ok, target_point) = K.Forward([math.pi*0.1, 0.0, 0.0, 0.0, 0.0])
        (ok, res_pos, frame) = K.Inverse(target_point, [0.0, 0.0, 0.0, 0.0, 0.0])


        traj_0 = GenTraj.GenFowardTraj(res_pos,4.0,  j_0)
        self.pub_traj.publish(traj_0)
        kin = GT.Kinematics()
        

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

# TODO : delete everything that is commented out in this function
#        self.bag = rosbag.Bag('mmuav_gazebo.bag', 'w')             ----- ROS is bugged, bagging from Python (the normal way) does not work

#        rosbag_proc = BH.StartBagRecordAll()
#        try:
#            self.PubInit()
#            self.AcqRoutine()
#            
#

#        except:
#            print("Rospy shut down or Gazebo not runing")
#            self.msgs[tags.index('unity_flag_port')] = 'q'

        

        self.AcqRoutine()

#        BH.StopBagRecordAll( rosbag_proc )


#        self.bag.close()                                           ----- ROS is bugged, bagging from Python (the normal way) does not work
if __name__ == '__main__':
    try:
        SP = SubPub()
        SP.Run()
        #rospy.spin()

    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException")
        pass    

