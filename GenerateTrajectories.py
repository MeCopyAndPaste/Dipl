#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from geometry_msgs.msg import Pose
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose                                                              
from geometry_msgs.msg import Vector3                                                           
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

from std_msgs.msg import String                                                                 # ROS messages needed for subs/pubs
from rosgraph_msgs.msg import Clock                                                             #
from gazebo_msgs.msg import LinkStates                                                          #
from sensor_msgs.msg import Joy                                                                 #
from geometry_msgs.msg import Pose                                                              #
from geometry_msgs.msg import Vector3                                                           #
from trajectory_msgs.msg import JointTrajectoryPoint                                            #
from trajectory_msgs.msg import JointTrajectory                                                 #
from std_msgs.msg import Header                                                                 #
#from trac_ik_python.trac_ik import IK
#from trac_ik_python.trac_ik_wrap import TRAC_IK

from tf.transformations import euler_from_quaternion, quaternion_from_euler                     # Modules needed for data processing

#################################################################################################
#  Robot parameters:

#x_0   = 0.0                                                                                     # Robot position and orientation at spawn
#y_0   = 0.0                                                                                     #
#z_0   = 1.0                                                                                     #
#yaw_0 = 0.0    
#j_0   = [0.0,0.0,0.0,0.0,0.0]                            # joints initial state

class Kinematics:
    

    def DHToFrame(self, a, alpha, d, theta):
        """Generates as PyKDL Frame using DH parameters."""
        M1 = PyKDL.Vector( math.cos(theta), -1.0*math.cos(alpha)*math.sin(theta),      math.sin(alpha)*math.sin(theta) )
        M2 = PyKDL.Vector( math.sin(theta),      math.cos(alpha)*math.cos(theta), -1.0*math.sin(alpha)*math.cos(theta) )
        M3 = PyKDL.Vector(              0,                       math.sin(alpha),                      math.cos(alpha) )
        M  = PyKDL.Rotation(x = M1, y = M2, z = M3)
        p  = PyKDL.Vector( a*math.cos(theta), a*math.sin(theta), d)

        return PyKDL.Frame(M, p)

    def FrameToPoint(self, frame):
        """Input is a PyKDL Frame. Returns a list of doubles; [x, y, z, r, p, y]."""

        (yaw, pitch, roll) = frame.M.GetEulerZYX()
        return [frame.p.x(), frame.p.y(), frame.p.z(), roll, pitch, yaw]
        
    def PointToFrame(self, target):
        """Input is a list of doubles; [x, y, z, r, p, y]. Returns PyKDL Frame."""

        tmp_M = PyKDL.Rotation.RPY(target[3], target[4], target[5])
        tmp_p = PyKDL.Vector(target[0], target[1], target[2])  

        return PyKDL.Frame(tmp_M, tmp_p)

    def GenChain(self):
        """Creates a KDL kinematic chain. Chain represents the manipulator mounted on the quadrotor drone. Chain is generated using hardoded DH parameters"""
        
        chain = PyKDL.Chain()

        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.075))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotX),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.12249))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotX),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.1365))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.075511))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.072489))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.04526))))

        return chain

    def Gen5DOFChain(self):
        """Creates a KDL kinematic chain. Chain represents the manipulator mounted on the quadrotor drone. Chain is generated using hardoded DH parameters"""
        
        chain = PyKDL.Chain()

        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.Fixed),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.075))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotX),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.12249))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotX),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.1365))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.075511))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.072489))))
        chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY),
                                      PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.04526))))

        return chain

    def ArrayToJntArray(self, array):
        jntarr = PyKDL.JntArray(len(array))
        for i in range(len(array)):
            jntarr[i] = array[i]

        return jntarr

    def ArrayToJntArrayVel(self, q, q_dot):
        jntarr = PyKDL.JntArrayVel(self.ArrayToJntArray(q),self.ArrayToJntArray(q_dot))
            
        return jntarr

    def CLBCKVelSolve(self, q, q_dot):
        chain           = self.GenChain()
        jntvelarr       = self.ArrayToJntArrayVel([0.0, q[0], q[1], q[2], q[3], q[4]], [0.0, q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4]])
        solver_VelSolve = PyKDL.ChainFkSolverVel_recursive(chain)
        FVel            = PyKDL.FrameVel()

        ok    = solver_VelSolve.JntToCart(jntvelarr, FVel)
        if ok < 0:
            raise Exception("ChainFkSolverVel in Kinematics.VelSolve can not find solution.") 

        Twist = FVel.GetTwist()        
        
        velarr = Twist.vel
        rotarr = Twist.rot
        return [velarr[0], velarr[1], velarr[2], rotarr[0], rotarr[1], rotarr[2]]

class TrajectoryPlanning:

    def __init__(self):
        pass

    def GenSinRoutineTraj(self, seq, ax, amplitudes, period, j_0):
        """Generates a JointTrajectoryPoint message for UAV motion in acquisition routine. 
           UAV motion involves motion along: X-axis, Y-axis, Z-axis (height above ground), and yaw motion"""

        traj                 = JointTrajectory()
        traj.header.seq      = seq
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()
        
        Hz          = 100                                                                       # Number of traj points per second as required by the controller
        T           = float(period)                                                             # casting to avoid possible error that could be 
        w           = 2*math.pi*(1/T)                                                           #        
        small_pause = 1.0                                                                       # Pause between fowards and backwards movements on same degree of freedom
        big_pause   = 1.0                                                                       # Pause between movements ivolving different degrees of freedom
        points_no   = int((T + small_pause + big_pause)*Hz)                                     #
        t_s         = np.linspace(0, period+small_pause+big_pause, points_no)                   #

        last_t = t_s[-1]
        count  = 0.0

        for axis in ax:
            #v = []                                                                              # Used for debugging
            #s = []                                                                              # Used for debugging
            #a = []                                                                              # Used for debugging
            
            k = ax.index(axis)
            A = float(amplitudes[k])

            if not(axis == 'x' or axis == 'y' or axis == 'z' or axis == 'yaw' or axis == 'man_yaw', axis == 'man_pitch', axis == 'man_pitch_base'):
                continue

            for i in range(len(t_s)):
                tmp_point   = JointTrajectoryPoint()

                if (t_s[i] >= T/2.0 and t_s[i] <= (T/2.0 + small_pause)):                       # Setting values for the points of desired trajectory
                    s_s = 2.0*A/w  
                    v_s = 0.0                                                                   # which include position, speed and acceleration
                    a_s = 0.0                                                                   # 
                                                                                                # 
                elif t_s[i] >= (T + small_pause):                                               # 
                    s_s = 0.0                                                                   # 
                    v_s = 0.0                                                                   # 
                    a_s = 0.0                                                                   # 
                                                                                                #  
                elif t_s[i] < T/2.0:                                                            #
                    v_s = A   * math.sin( w*t_s[i] )                                            #  
                    s_s =-A/w * math.cos( w*t_s[i] ) + A/w                                      # 
                    a_s = A*w * math.cos( w*t_s[i] )                                            # 
                                                                                                #
                elif t_s[i] > (T/2.0 + small_pause) and t_s[i] < (T + small_pause):                             #  
                    v_s = A   * math.sin( w*(t_s[i] - small_pause))                             #  
                    s_s =-A/w * math.cos( w*(t_s[i] - small_pause)) + A/w                       # 
                    a_s = A*w * math.cos( w*(t_s[i] - small_pause))                             #
 
                if axis == 'x':
                    #                          x           y         z             dummy   dummy1  yaw         joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [j_0[0]+s_s, j_0[1]    , j_0[2]    , j_0[3], j_0[4], j_0[5]    , j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]]
                    tmp_point.velocities    = [v_s       , 0.0       , 0.0       , 0.0   , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
                    tmp_point.accelerations = [a_s       , 0.0       , 0.0       , 0.0   , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
                elif axis == 'y':
                    #                          x           y           z           dummy  dummy1  yaw          joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [j_0[0]    , s_s+j_0[1], j_0[2]    , j_0[3], j_0[4], j_0[5]    , j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]]
                    tmp_point.velocities    = [0.0       , v_s       , 0.0       , 0.0   , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
                    tmp_point.accelerations = [0.0       , a_s       , 0.0       , 0.0   , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
 
                elif axis == 'z':
                    #                          x           y         z             dummy  dummy1  yaw          joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [j_0[0]    , j_0[1]    , s_s+j_0[2], j_0[3], j_0[4], j_0[5]    , j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]]
                    tmp_point.velocities    = [0.0       , 0.0       , v_s       , 0.0   , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
                    tmp_point.accelerations = [0.0       , 0.0       , a_s       , 0.0   , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]

                elif axis == 'yaw':
                    #                          x           y         z             dummy  dummy1  yaw          joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [j_0[0]    , j_0[1]    , j_0[2]    , j_0[3], j_0[4], s_s+j_0[5], j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]]
                    tmp_point.velocities    = [0.0       , 0.0       , 0.0       , 0.0   , 0.0   , v_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
                    tmp_point.accelerations = [0.0       , 0.0       , 0.0       , 0.0   , 0.0   , a_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]

                elif axis == 'man_yaw':
                    #                          x           y         z             dummy  dummy1  yaw          joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [j_0[0]    , j_0[1]    , j_0[2]    , j_0[3], j_0[4], j_0[5], j_0[6], j_0[7], s_s+j_0[8], j_0[9], j_0[10]]
                    tmp_point.velocities    = [0.0       , 0.0       , 0.0       , 0.0   , 0.0   , v_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
                    tmp_point.accelerations = [0.0       , 0.0       , 0.0       , 0.0   , 0.0   , a_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]

                elif axis == 'man_pitch':
                    #                          x           y         z             dummy  dummy1  yaw          joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [j_0[0]    , j_0[1]    , j_0[2]    , j_0[3], j_0[4], j_0[5], j_0[6], s_s+j_0[7], j_0[8], j_0[9], j_0[10]]
                    tmp_point.velocities    = [0.0       , 0.0       , 0.0       , 0.0   , 0.0   , v_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
                    tmp_point.accelerations = [0.0       , 0.0       , 0.0       , 0.0   , 0.0   , a_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]

                elif axis == 'man_pitch_base':
                    #                          x           y         z             dummy  dummy1  yaw          joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [j_0[0]    , j_0[1]    , j_0[2]    , j_0[3], j_0[4], j_0[5], s_s+j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]]
                    tmp_point.velocities    = [0.0       , 0.0       , 0.0       , 0.0   , 0.0   , v_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
                    tmp_point.accelerations = [0.0       , 0.0       , 0.0       , 0.0   , 0.0   , a_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0    ]
            
                tmp_point.time_from_start.secs  = int( t_s[i] + last_t * count)
                tmp_point.time_from_start.nsecs = int((t_s[i] + last_t * count)*1000000000 - int(t_s[i] + last_t * count)*1000000000)
                traj.points.append(tmp_point)
                
                #print("t_s    :", t_s[i])                                                       # Used for debugging
                #print("last_t :", last_t)                                                       # Used for debugging
                #print("count  :", count)                                                        # Used for debugging
                #print("secs   :", tmp_point.time_from_start.secs)                               # Used for debugging
                #print("nsecs  :", tmp_point.time_from_start.nsecs)                              # Used for debugging

            
                #v.append(v_s)                                                                   # Used for debugging
                #s.append(s_s)                                                                   # Used for debugging
                #a.append(a_s)                                                                   # Used for debugging
                #print(t_s[i])                                                                   # Used for debugging 

            count = count + 1.0                                                                

            #plt.figure(int(count))                                                              # Used for debugging
            #plt.plot(t_s, s, t_s, v, t_s, a)                                                    # Used for debugging
            #plt.axis('equal')                                                                   # Used for debugging
            #plt.show()                                                                          # Used for debugging

        return traj

    def GenJointsManRoutine(self, amplitudes, period, j_0):
        """Generates a JointTrajectoryPoint message for UAV's manipulator arm in acquisition routine."""

        traj                 = JointTrajectory()
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()

        Hz          = 100                                                                       # Number of traj points per second as required by the controller
        T           = float(period)                                                             # 
        w           = 2*math.pi*(1/T)                                                           #        
        small_pause = 4.0                                                                       # Pause between fowards and backwards movements on same degree of freedom
        big_pause   = 4.0                                                                       # Pause between movements ivolving different degrees of freedom
        points_no   = int((T + small_pause + big_pause)*Hz)                                     #
        t_s         = np.linspace(0, T+small_pause+big_pause, points_no)                        #

        last_t      = t_s[-1]
        count       = 0.0

        for joint in traj.joint_names:
            v = []                                                                              # Used for debugging
            s = []                                                                              # Used for debugging
            a = []                                                                              # Used for debugging
            
            k = traj.joint_names.index(joint)
            A = float(amplitudes[k])

            if not(joint == 'joint1' or joint == 'joint2' or joint == 'joint3' or joint == 'joint4' or joint == 'joint5'):
                continue

            for i in range(len(t_s)):
                tmp_point   = JointTrajectoryPoint()

                if (t_s[i] >= T/2.0 and t_s[i] <= (T/2.0 + small_pause)):                       # Setting values for the points of desired trajectory
                    s_s = A   
                    v_s = 0.0                                                                   # which include position, speed and acceleration
                    a_s = 0.0                                                                   # 
                                                                                                # 
                elif t_s[i] >= (T + small_pause):                                               # 
                    s_s = 0.0                                                                   # 
                    v_s = 0.0                                                                   # 
                    a_s = 0.0                                                                   # 
                                                                                                #  
                elif t_s[i] < T/2.0:                                                            #
                    s_s =-A/2.0 * math.cos( w*t_s[i] ) + A/2.0                                  #
                    v_s = 0.0                                                                   # 
                    a_s = 0.0                                                                   # 
                                                                                                #
                elif t_s[i] > (T/2.0 + small_pause) and t_s[i] < (T + small_pause):             #  
                    s_s =-A/2.0 * math.cos( w*(t_s[i] - small_pause)) + A/2.0                   # 
                    v_s = 0.0                                                                   # 
                    a_s = 0.0                                                                   # 
                if  joint == 'joint1':
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], s_s+j_0[6], j_0[7]    , j_0[8]     , j_0[9]    , j_0[10]     ]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                elif joint == 'joint2': 
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], j_0[6]    , s_s+j_0[7], j_0[8]     , j_0[9]    , j_0[10]     ]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                elif joint == 'joint3':
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], j_0[6]    , j_0[7]    , s_s+j_0[8] , j_0[9]    , j_0[10]     ]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]

                elif joint == 'joint4':
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], j_0[6],     j_0[7],     j_0[8]     , s_s+j_0[9], j_0[10]     ]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
            
                elif joint == 'joint5':
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], j_0[6]    , j_0[7]    , j_0[8]     , j_0[9]    , s_s+j_0[10]]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0        ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0        ]
    
                tmp_point.time_from_start.secs  = int( t_s[i] + last_t * count)
                tmp_point.time_from_start.nsecs = int((t_s[i] + last_t * count)*1000000000 - int(t_s[i] + last_t * count)*1000000000)
                traj.points.append(tmp_point)
                
                #print("t_s    :", t_s[i])                                                       # Used for debugging
                #print("last_t :", last_t)                                                       # Used for debugging
                #print("count  :", count)                                                        # Used for debugging
                #print("secs   :", tmp_point.time_from_start.secs)                               # Used for debugging
                #print("nsecs  :", tmp_point.time_from_start.nsecs)                              # Used for debugging

            
                v.append(v_s)                                                                   # Used for debugging
                s.append(s_s)                                                                   # Used for debugging
                a.append(a_s)                                                                   # Used for debugging
                print(t_s[i])                                                                   # Used for debugging 

            count = count + 1.0                                                                

            plt.figure(int(count))                                                              # Used for debugging
            plt.plot(t_s, s, t_s, v, t_s, a)                                                    # Used for debugging
            plt.axis('equal')                                                                   # Used for debugging
            plt.show()                                                                          # Used for debugging 

        return traj

    def GoToPoint(self, seq , NoRot, time, j_0, target_pos, Force5DOF = False, step = 0.0001):
        """ """

        K  = Kinematics()
        TP = TrajectoryPlanning()
        frame = PyKDL.Frame.Identity()
        if Force5DOF == True:
            chain = K.Gen5DOFChain()
            q_max1 = K.ArrayToJntArray([ 1.57,  2.2,  1.57,  2,  2.3])
            q_min1 = K.ArrayToJntArray([-1.57, -2.2, -1.57, -2, -2.3])            
        else:
            q_max1 = K.ArrayToJntArray([ 1.57,  1.57,  2.2,  1.6,  2,  2.3])
            q_min1 = K.ArrayToJntArray([-1.57, -1.57, -2.2, -1.6, -2, -2.3])
            chain = K.GenChain()


        solver_fk = PyKDL.ChainFkSolverPos_recursive(chain)
        solver_vel = PyKDL.ChainIkSolverVel_pinv_givens(chain)
        solver_ik = PyKDL.ChainIkSolverPos_NR_JL(chain, q_min1, q_max1, solver_fk, solver_vel)

        traj                 = JointTrajectory()
        traj.header.seq      = seq
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()
        traj_tmp             = JointTrajectory() 
        traj_tmp.header      = traj.header 
        traj_tmp.joint_names = traj.joint_names  
        #j_0                  = [ 0.0, 0.0, 1.0, 0.0, 0,0, 0.0, 0.0, 0,0, 0.0, 0.0]

        if Force5DOF == True:
            q_init   = K.ArrayToJntArray([j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]])
        else:
            q_init   = K.ArrayToJntArray([j_0[5], j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]])
        
        frame    = PyKDL.Frame.Identity()
        ok_fk    = solver_fk.JntToCart(q_init, frame, -1)
        cp       = K.FrameToPoint(frame)

        step_lin = step
        #step_lin = 0.0001
        step_rot = step_lin*20
        tp       = target_pos
        last_p   = cp
       #last_p   = start_pos
       #cp       = start_pos
 
        distance_lin = math.sqrt(abs(cp[0]-tp[0])**2 + abs(cp[1]-tp[1])**2 + abs(cp[2]-tp[2])**2)
        distance_rot = math.sqrt(abs(cp[3]-tp[3])**2 + abs(cp[4]-tp[4])**2 + abs(cp[5]-tp[5])**2)

        while distance_lin >= step_lin or (distance_rot >= step_rot and not NoRot):
            q_s = PyKDL.JntArray(chain.getNrOfJoints())
                        
            if distance_lin == 0.0 :
                a = 0.0
                b = 0.0
                c = 0.0
            else:
                a = step_lin * (tp[0]-cp[0])/distance_lin
                b = step_lin * (tp[1]-cp[1])/distance_lin
                c = step_lin * (tp[2]-cp[2])/distance_lin

            if distance_rot == 0.0 or NoRot:
                r = 0.0
                p = 0.0
                y = 0.0
            else:                
                r = step_rot * (tp[3]-cp[3])/distance_rot
                p = step_rot * (tp[4]-cp[4])/distance_rot
                y = step_rot * (tp[5]-cp[5])/distance_rot

            cp[0] = cp[0] + a
            cp[1] = cp[1] + b
            cp[2] = cp[2] + c
            cp[3] = cp[3] + r
            cp[4] = cp[4] + p
            cp[5] = cp[5] + y

            
            target_frame = K.PointToFrame(cp)
            #print q_init

            for i in range(q_init.rows()):
                q_init[i] = q_init[i]+0.001 #q[i]+0.1*random.random()

            #print  [distance_lin, distance_rot]

            ok = solver_ik.CartToJnt(q_init, target_frame, q_s)

            if ok < 0:
                raise Exception("Inverse kinematics solver failed") 
                #print("\nSolver failed. Solver returned value: %s" % (ok))
                #print "q_init:"
                #print q_init
                #break

            #target_frame1 = PyKDL.Frame.Identity()
            #solver_fk.JntToCart(q_s,target_frame1,-1)

            last_p = cp
            q_init = q_s
            distance_lin = math.sqrt(abs(cp[0]-tp[0])**2 + abs(cp[1]-tp[1])**2 + abs(cp[2]-tp[2])**2)
            distance_rot = math.sqrt(abs(cp[3]-tp[3])**2 + abs(cp[4]-tp[4])**2 + abs(cp[5]-tp[5])**2)


            tmp_point   = JointTrajectoryPoint()
            t_s = []
            
            if Force5DOF == True:
                q_t= [0.0,  q_s[0] ,q_s[1] ,q_s[2] ,q_s[3] ,q_s[4]]
            else:
                q_t = q_s
            #                          x      y      z         dummy   dummy1  yaw                  joint1        joint2        joint3        joint4        joint5
            tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5]+q_t[0]*(-1.0), q_t[1]       ,q_t[2]       ,q_t[3]       ,q_t[4]       ,q_t[5]        ]
            tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0                 , 0.0          , 0.0         , 0.0         , 0.0         , 0.0          ]
            tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0                 , 0.0          , 0.0         , 0.0         , 0.0         , 0.0          ]
    
            tmp_point.time_from_start.secs  = 0.0 #int( t_s[i])
            tmp_point.time_from_start.nsecs = 0.0 #int(t_s[i] *1000000000 - int(t_s[i])*1000000000)
            traj_tmp.points.append(tmp_point)




        #plt.figure(int())                                                              # Used for debugging
        #plt.plot(range(len(traj.points)), plotval)
        #plt.legend(traj.joint_names)                                                   # Used for debugging
        #plt.axis('equal')                                                                   # Used for debugging
        #plt.show()

        if Force5DOF == True:
            q_o = [0.0, q_init[0], q_init[1], q_init[2], q_init[3], q_init[4]]

        print "\nlast joints:"    
        print q_o 

        return q_o, traj_tmp

    def InitPosition(self, seq, j_0, q_target):
        """Moves manipulator joints to position specified by q_target. It takes 5 seconds to complete the motion. q_target is either array or PyKDL.JntArray, both works."""
        traj                 = JointTrajectory()
        traj.header.seq      = seq
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()
       #j_0                  = [ 0.0, 0.0, 1.0, 0.0, 0,0, 0.0, 0.0, 0,0, 0.0, 0.0]

        time = 5.0
        t_s  = np.linspace(0, time, int(time*100))                        #

        q_init = [j_0[5], j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]]        
        q_max = [2.0*(q_target[i]-q_init[i])/time  for i in range(len(q_init))] 

        for i in range(len(t_s)):
            tmp_point   = JointTrajectoryPoint()

            if t_s[i] < time/2 :
                q_s = [q_max[j]/time*t_s[i]**2  for j in range(len(q_max))]
            else:
                q_s = [-q_max[j]/time*t_s[i]**2 + 2.0*q_max[j]*t_s[i] - 0.5*q_max[j]*time for j in range(len(q_max))]


            #                          x      y      z         dummy   dummy1  yaw     joint1        joint2        joint3        joint4        joint5
            tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], q_s[1]+j_0[6],q_s[2]+j_0[7],q_s[3]+j_0[8],q_s[4]+j_0[9],q_s[5]+j_0[10]]
            tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0          , 0.0         , 0.0         , 0.0         , 0.0          ]
            tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0          , 0.0         , 0.0         , 0.0         , 0.0          ]
    
            tmp_point.time_from_start.secs  = int(t_s[i])
            tmp_point.time_from_start.nsecs = int(t_s[i] *1000000000 - int(t_s[i])*1000000000)
            traj.points.append(tmp_point)
        
        return traj

    def GenDOFManRoutine(self, seq, amplitudes, period, j_0):
        """ """
        traj                 = JointTrajectory()
        traj.header.seq      = seq
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()

        Hz          = 100                                                                       # Number of traj points per second as required by the controller
        T           = float(period)                                                             # 
        w           = 2*math.pi*(1/T)                                                           #        
        small_pause = 4.0                                                                       # Pause between fowards and backwards movements on same degree of freedom
        big_pause   = 4.0                                                                       # Pause between movements ivolving different degrees of freedom
        points_no   = int((T + small_pause + big_pause)*Hz)                                     #
        t_s         = np.linspace(0, T+small_pause+big_pause, points_no)                        #

        last_t      = t_s[-1]
        count       = 0.0

        for joint in traj.joint_names:
            v = []                                                                              # Used for debugging
            s = []                                                                              # Used for debugging
            a = []                                                                              # Used for debugging
            
            k = traj.joint_names.index(joint)
            A = float(amplitudes[k])

            if not(joint == 'joint1' or joint == 'joint2' or joint == 'joint3' or joint == 'joint4' or joint == 'joint5'):
                continue

            for i in range(len(t_s)):
                tmp_point   = JointTrajectoryPoint()

                if (t_s[i] >= T/2.0 and t_s[i] <= (T/2.0 + small_pause)):                       # Setting values for the points of desired trajectory
                    s_s = A   
                    v_s = 0.0                                                                   # which include position, speed and acceleration
                    a_s = 0.0                                                                   # 
                                                                                                # 
                elif t_s[i] >= (T + small_pause):                                               # 
                    s_s = 0.0                                                                   # 
                    v_s = 0.0                                                                   # 
                    a_s = 0.0                                                                   # 
                                                                                                #  
                elif t_s[i] < T/2.0:                                                            #
                    s_s =-A/2.0 * math.cos( w*t_s[i] ) + A/2.0                                  #
                    v_s = 0.0                                                                   # 
                    a_s = 0.0                                                                   # 
                                                                                                #
                elif t_s[i] > (T/2.0 + small_pause) and t_s[i] < (T + small_pause):             #  
                    s_s =-A/2.0 * math.cos( w*(t_s[i] - small_pause)) + A/2.0                   # 
                    v_s = 0.0                                                                   # 
                    a_s = 0.0                                                                   # 
                if  joint == 'joint1':
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], s_s+j_0[6], j_0[7]    , j_0[8]     , j_0[9]    , j_0[10]     ]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                elif joint == 'joint2': 
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], j_0[6]    , s_s+j_0[7], j_0[8]     , j_0[9]    , j_0[10]     ]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                elif joint == 'joint3':
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], j_0[6]    , j_0[7]    , s_s+j_0[8] , j_0[9]    , j_0[10]     ]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]

                elif joint == 'joint4':
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], j_0[6],     j_0[7],     j_0[8]     , s_s+j_0[9], j_0[10]     ]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0         ]
            
                elif joint == 'joint5':
                    #                          x      y      z         dummy   dummy1  yaw     joint1      joint2      joint3      joint4       joint5
                    tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], j_0[6]    , j_0[7]    , j_0[8]     , j_0[9]    , s_s+j_0[10]]
                    tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0        ]
                    tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0       , 0.0       , 0.0        , 0.0       , 0.0        ]
    
                tmp_point.time_from_start.secs  = int( t_s[i] + last_t * count)
                tmp_point.time_from_start.nsecs = int((t_s[i] + last_t * count)*1000000000 - int(t_s[i] + last_t * count)*1000000000)
                traj.points.append(tmp_point)
                
                #print("t_s    :", t_s[i])                                                       # Used for debugging
                #print("last_t :", last_t)                                                       # Used for debugging
                #print("count  :", count)                                                        # Used for debugging
                #print("secs   :", tmp_point.time_from_start.secs)                               # Used for debugging
                #print("nsecs  :", tmp_point.time_from_start.nsecs)                              # Used for debugging

            
                v.append(v_s)                                                                   # Used for debugging
                s.append(s_s)                                                                   # Used for debugging
                a.append(a_s)                                                                   # Used for debugging
                print(t_s[i])                                                                   # Used for debugging 

            count = count + 1.0                                                                

            plt.figure(int(count))                                                              # Used for debugging
            plt.plot(t_s, s, t_s, v, t_s, a)                                                    # Used for debugging
            plt.axis('equal')                                                                   # Used for debugging
            plt.show()                                                                          # Used for debugging 

        return traj

    def FlipTraj(self, seq, traj):
        flipped_traj = JointTrajectory() 
        flipped_traj.header.seq = seq
        flipped_traj.header = traj.header
        flipped_traj.joint_names = traj.joint_names

        for i in range(len(traj.points)):
            flipped_traj.points.insert(0, traj.points[i])
            
    
        for i in range(len(traj.points)):
            flipped_traj.points[i].time_from_start = traj.points[i].time_from_start

        return flipped_traj



if __name__ == '__main__':
    rospy.init_node('Trajectory_Generator', anonymous=True)
    
    # K  = Kinematics()
    # TP = TrajectoryPlanning()
    
    # frame = PyKDL.Frame.Identity()
    # target_frame = PyKDL.Frame.Identity()

    # chain = K.GenChain()

    # q_max1 = K.ArrayToJntArray([1.57, 1.57, 2.2, 1.6, 2, 2.3])
    # q_min1 = K.ArrayToJntArray([-1.57, -1.57, -2.2, -1.6, -2, -2.3])

    # solver_fk = PyKDL.ChainFkSolverPos_recursive(chain)
    # solver_vel = PyKDL.ChainIkSolverVel_pinv_givens(chain)
    # solver_ik = PyKDL.ChainIkSolverPos_NR_JL(chain, q_min1, q_max1, solver_fk, solver_vel)

    # q_start = [0.0, math.pi/6, math.pi/3, -0.436343, 0.892081, -0.455771]
    # q_start = K.ArrayToJntArray(q_start)
    # # q_start = [0.0, -0.31276, 0.197715, 1.57, 0, 0]
    # # #q_start = [  -0.0748982, -0.00272102,     1.57352,   -0.628087,       1.654,    -0.95101,]
    # # #last_jnts = [  -0.0748982, -0.00272102,     -2.0,   -0.628087,       1.654,    -0.95101]
    # # #[ 0.0, 0.0, 1.0, 0.0, 0,0.0, 0.0, 1.57, -0.798059, 1.63987, -0.841811]
    # # q_start = [0.0, 0.0, 1.57, -0.798059, 1.63987, -0.841811]

    # # last_jnts = q_start
    # # #j_0 = [ 0.0, 0.0, 1.0, 0,0, last_jnts[0], last_jnts[1], last_jnts[2], last_jnts[3], last_jnts[4], last_jnts[5]]
    # # j_0 = [ 0.0, 0.0, 1.0, 0,0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # # #traj = TP.InitPosition(j_0, [[  0., 0. ,1.57,   -0.610121,     1.66769,   -0.972666]]])


    # # traj = TP.InitPosition(j_0, q_start)
    # j_0 = [ 0.0, 0.0, 1.0, 0,0, q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]]

    
    # q_init = K.ArrayToJntArray(q_start)                                     # initial position fk
    # ok = solver_fk.JntToCart(q_start, frame, -1)                             #
    # start_point = K.FrameToPoint(frame)                                     #
    # print start_point
    # #                  x              y        z                   r     p    y
    # target_point = [0.0, -0.2797, 0.19749, 1.5707963267948966, 0.0, 0.0]
    # target_frame = K.PointToFrame(target_point)

    # print "\nstart_point"
    # print start_point
    # print "target_point"
    # print target_point
    # print "\n\n\n"


    # # """
    # # q_target = []
    # # ok = solver_fk.JntToCart(q_target, target_frame, -1)
    # # target_point = K.FrameToPoint(target_frame)
    # # """
    # traj_to_pos = TP.GoToPoint(seq = 1, NoRot = True, time = 4.0, j_0 = j_0, target_pos = target_point, Force5DOF = False)
    # # #traj_to_pos = TP.FlipTraj(TP.GoToPoint(chain, solver_fk, solver_ik, 4.0, j_0, target_point))
    
    # # pub = rospy.Publisher('/uav/joint_trajectory', JointTrajectory, queue_size=10)
    # # rate = rospy.Rate(10) # 10hz
    # # rate.sleep()
    # # first = 1

    
    # # while not rospy.is_shutdown():
    # #     if first == 1:
    # #         first = 0
    # #         pub.publish(traj)
    # #     rate.sleep()
    
    




