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

x_0   = 0.0                                                                                     # Robot position and orientation at spawn
y_0   = 0.0                                                                                     #
z_0   = 1.0                                                                                     #
yaw_0 = 0.0    
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

        #tmp_1 = PyKDL.Rotation.RotX(target[3])
        #tmp_2 = PyKDL.Rotation.RotX(target[4])
        #tmp_3 = PyKDL.Rotation.RotX(target[5])
        # Xx =      math.cos(target[3])*math.cos(target[4])
        # Yx = -1.0*math.sin(target[3])*math.cos(target[5]) + math.cos(target[3])*math.sin(target[4])*math.sin(target[5])
        # Zx =      math.sin(target[3])*math.sin(target[5]) + math.cos(target[3])*math.sin(target[4])*math.cos(target[5])
        # Xy =      math.sin(target[3])*math.cos(target[4])
        # Yy =      math.cos(target[3])*math.cos(target[5]) + math.sin(target[3])*math.sin(target[4])*math.sin(target[5])
        # Zy = -1.0*math.cos(target[3])*math.sin(target[5]) + math.sin(target[3])*math.sin(target[4])*math.cos(target[5])
        # Xz = -1.0*math.sin(target[4])
        # Yz =      math.cos(target[4])*math.sin(target[5])
        # Zz =      math.cos(target[4])*math.cos(target[5])

        tmp_M = PyKDL.Rotation()
        
        #tmp_1.DoRotX(target[3])
        #tmp_2.DoRotY(target[4])
        #tmp_3.DoRotZ(target[5])
        
        #tmp_M = tmp_3 * tmp_2 * tmp_1
        tmp_M.RPY(target[3], target[4], target[5])
        tmp_p = PyKDL.Vector(target[0], target[1], target[2])  
        
        return PyKDL.Frame(tmp_M, tmp_p)

    def GenChain(self):
        """Creates a KDL kinematic chain. Chain represents the manipulator mounted on the quadrotor drone. Chain is generated using hardoded DH parameters"""
        
        chain = PyKDL.Chain()

        f = self.DHToFrame(0.0, -1.0*math.pi, 0.0353, 0.0)
        joint_tmp   = PyKDL.Joint(name = "joint0",  type = PyKDL.Joint.Fixed)
        segment_tmp = PyKDL.Segment(name = u"base", joint =joint_tmp, f_tip = self.DHToFrame(a = 0.0, alpha = -0.5*math.pi, d = 0.0353, theta = 0.0))        
        chain.addSegment(segment_tmp)

        joint_tmp   = PyKDL.Joint(name = "joint1",  type = PyKDL.Joint.RotZ)
        segment_tmp = PyKDL.Segment(name = u"segment1", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.1225, alpha = 0.0, d = 0.0, theta = 0.5*math.pi))        
        chain.addSegment(segment_tmp)
        
        joint_tmp   = PyKDL.Joint(name = "joint2",  type = PyKDL.Joint.RotZ)
        segment_tmp = PyKDL.Segment(name = u"segment2", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.1365, alpha = -0.5*math.pi, d = 0.0, theta = math.pi ))        
        chain.addSegment(segment_tmp)

        joint_tmp   = PyKDL.Joint(name = "joint3",  type = PyKDL.Joint.RotZ)
        segment_tmp = PyKDL.Segment(name = u"segment3", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.0755, alpha = 0.0, d = 0.0, theta = 0.0))        
        chain.addSegment(segment_tmp)
        
        joint_tmp   = PyKDL.Joint(name = "joint4",  type = PyKDL.Joint.RotZ)
        segment_tmp = PyKDL.Segment(name = u"segment4", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.0725, alpha = 0.0, d = 0.0, theta = 0.0))        
        chain.addSegment(segment_tmp)

        joint_tmp   = PyKDL.Joint(name = "joint5",  type = PyKDL.Joint.RotZ)
        segment_tmp = PyKDL.Segment(name = u"segment5", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.0353, alpha = 0.0, d = 0.0, theta = 0.0))        
        chain.addSegment(segment_tmp)
        
        joint_tmp   = PyKDL.Joint(name = "v_jnt1",  type = PyKDL.Joint.Fixed)
        segment_tmp = PyKDL.Segment(name = u"virtual1", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.0, alpha = 0.0, d = 0.0, theta = 0.5*math.pi))        
        chain.addSegment(segment_tmp)

        joint_tmp   = PyKDL.Joint(name = "v_jnt2",  type = PyKDL.Joint.Fixed)
        segment_tmp = PyKDL.Segment(name = u"virtual12", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.0, alpha = 0.5*math.pi, d = 0.0, theta = 0.0))        
        chain.addSegment(segment_tmp)

        joint_tmp   = PyKDL.Joint(name = "tool_fixed_jnt",  type = PyKDL.Joint.Fixed)
        segment_tmp = PyKDL.Segment(name = u"tool", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.0, alpha = 0.0, d = 0.32, theta = -0.5*math.pi))        
        chain.addSegment(segment_tmp)

        return chain

    def Forward(self, joints):
        """Description not implemented"""

        tool_frame = PyKDL.Frame()
        chain      = self.GenChain()
        jnt_arr    = PyKDL.JntArray(chain.getNrOfJoints())
        jnt_arr[0] = joints[0]
        jnt_arr[1] = joints[1]
        jnt_arr[2] = joints[2]
        jnt_arr[3] = joints[3]
        jnt_arr[4] = joints[4]
        solver_fk  = PyKDL.ChainFkSolverPos_recursive(chain)
        
        ok = solver_fk.JntToCart(jnt_arr, tool_frame, -1)
        tool_xyt_rpy = self.FrameToPoint(tool_frame)

        #self.PlotArm(solver_fk, jnt_arr)

        return [ok, tool_xyt_rpy]

    def Inverse(self, chain, fk_res_frame, current_joint_positions):
        """Description not implemented"""

        fk_res_frame = PyKDL.Frame()
        #robot from DH parameters:
        #chain   = self.GenChain()
        ############################################################################################################################################################
        #limits
        q_max = PyKDL.JntArray(chain.getNrOfJoints())
        q_min = PyKDL.JntArray(chain.getNrOfJoints())        
        q_max[0] = 1.57
        q_max[1] = 2.2
        q_max[2] = 1.57
        q_max[3] = 1.57
        q_max[4] = 2.3
        q_min[0] =-1.57 
        q_min[1] =-2.2
        q_min[2] =-1.57
        q_min[3] =-1.57
        ############################################################################################################################################################
        #creating solvers:
        solver_fk  = PyKDL.ChainFkSolverPos_recursive(chain)
        solver_vel = PyKDL.ChainIkSolverVel_pinv_givens(chain)
        solver_ik  = PyKDL.ChainIkSolverPos_NR_JL(chain, q_min, q_max, solver_fk, solver_vel)
        ###############################################################################################################################
        #joint arrays and PyKDL geometric primitives:
        jnt_end_pos   = PyKDL.JntArray(chain.getNrOfJoints())
        jnt_start_pos = PyKDL.JntArray(chain.getNrOfJoints())
        ############################################################################################################################################################
        #converting joint lists into PyKDL joint arrays:
        jnt_start_pos[0] = current_joint_positions[0]
        jnt_start_pos[1] = current_joint_positions[1]
        jnt_start_pos[2] = current_joint_positions[2]
        jnt_start_pos[3] = current_joint_positions[3]
        jnt_start_pos[4] = current_joint_positions[4]
        ############################################################################################################################################################
        #solving kinematics problem:
        #Kin = Kinematics()
        #fk_res_frame = Kin.PointToFrame(tool_xyt_rpy)
        #print fk_res_frame
        ok = solver_ik.CartToJnt(jnt_start_pos, fk_res_frame, jnt_end_pos)
        ############################################################################################################################################################
        #converting joint lists into PyKDL joint arrays:
        joints = []

        joints.append(jnt_end_pos[0])
        joints.append(jnt_end_pos[1])
        joints.append(jnt_end_pos[2])
        joints.append(jnt_end_pos[3])
        joints.append(jnt_end_pos[4])

        #self.PlotArm(solver_fk, jnt_end_pos)
        fk_res_frame
        f4 = PyKDL.Frame()
        f5 = PyKDL.Frame()
        f6 = PyKDL.Frame()
        f7 = PyKDL.Frame()
        f8 = PyKDL.Frame()
        f9 = PyKDL.Frame()
        f10= PyKDL.Frame()
        ok = solver_fk.JntToCart(jnt_end_pos, f1, 0)
        ok = solver_fk.JntToCart(jnt_end_pos, f2, 1)
        ok = solver_fk.JntToCart(jnt_end_pos, f3, 2)
        ok = solver_fk.JntToCart(jnt_end_pos, f4, 3)
        ok = solver_fk.JntToCart(jnt_end_pos, f5, 4)
        ok = solver_fk.JntToCart(jnt_end_pos, f6, 5)
        ok = solver_fk.JntToCart(jnt_end_pos, f7, 6)
        ok = solver_fk.JntToCart(jnt_end_pos, f8, 7)
        ok = solver_fk.JntToCart(jnt_end_pos, f9, 8)
        ok = solver_fk.JntToCart(jnt_end_pos,f10, 9)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_aspect('equal')

        x_s = [f1.p.x(), f2.p.x(), f3.p.x(), f4.p.x(), f5.p.x(), f6.p.x(), f7.p.x(), f8.p.x(), f9.p.x(),f10.p.x()]
        y_s = [f1.p.y(), f2.p.y(), f3.p.y(), f4.p.y(), f5.p.y(), f6.p.y(), f7.p.y(), f8.p.y(), f9.p.y(),f10.p.y()]
        z_s = [f1.p.z(), f2.p.z(), f3.p.z(), f4.p.z(), f5.p.z(), f6.p.z(), f7.p.z(), f8.p.z(), f9.p.z(),f10.p.z()]
        for i in range(len(x_s)):
            if abs(x_s[i]) < 0.0001:
                x_s[i] = 0.0
            if abs(y_s[i]) < 0.0001:
                y_s[i] = 0.0
            if abs(z_s[i]) < 0.0001:
                z_s[i] = 0.0

        for i in range(len(x_s)-1):
            ax.plot([x_s[i],x_s[i+1]],[y_s[i],y_s[i+1]],[z_s[i],z_s[i+1]])

        ax.scatter(x_s, y_s, z_s)

        txt = ["base","j1", "j2", "j3", "j4", "j5", "v1", "v2", "t1", "tip"]
        for i in range(len(x_s)):
            ax.text(x_s[i],y_s[i],z_s[i],  '%s' % txt[i], size=10, zorder=1,  color='k') 

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        plt.show()

    def ArrayToJntArray(self, array):
        jntarr = PyKDL.JntArray(len(array))
        for i in range(len(array)):
            jntarr[i] = array[i]

        return jntarr

    def ArrayToJntArrayVel(self, array):
        jntarr = PyKDL.JntArrayVel(len(array))
        
        for i in range(len(array)):
            jntarr[i] = array[i]
            
        return jntarr

class TrajectoryPlanning:

    def __init__(self):
        pass

    def VelSolve(self, JntVels):
        


    def GenSinXYZRoutineTraj(self, amplitudes, period, j_0):
        """Generates a JointTrajectoryPoint message for UAV motion in acquisition routine. 
           UAV motion involves motion along: X-axis, Y-axis, Z-axis (height above ground), and yaw motion"""

        traj                 = JointTrajectory()
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()
        
        Hz          = 100                                                                       # Number of traj points per second as required by the controller
        T           = float(period/2)                                                           # casting to avoid possible error that could be 
        w           = 2*math.pi*(1/T)                                                           #        
        small_pause = 2.0                                                                       # Pause between fowards and backwards movements on same degree of freedom
        big_pause   = 2.0                                                                       # Pause between movements ivolving different degrees of freedom
        points_no   = int((T + small_pause + big_pause)*Hz)                                     #
        t_s         = np.linspace(0, period+small_pause+big_pause, points_no)                   #

        last_t = t_s[-1]
        count  = 0.0

        for axis in traj.joint_names:
            #v = []                                                                              # Used for debugging
            #s = []                                                                              # Used for debugging
            #a = []                                                                              # Used for debugging
            
            k = traj.joint_names.index(axis)
            A = float(amplitudes[k])

            if not(axis == 'x' or axis == 'y' or axis == 'z' or axis == 'yaw'):
                continue

            for i in range(len(t_s)):
                tmp_point   = JointTrajectoryPoint()

                if (t_s[i] >= T/2.0 and t_s[i] <= (T/2.0 + small_pause)):                       # Setting values for the points of desired trajectory
                    s_s = A/w  
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
                elif t_s[i] > (T/2.0 + 2.0) and t_s[i] < (T + 2.0):                             #  
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

    def GoToPoint(self, chain, solver_fk, solver_ik, time, start_pos, target_pos):
        """ """
        traj                 = JointTrajectory()
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()
        j_0                  = [ 0.0, 0.0, 1.0, 0.0, 0,0, 0.0, 0.0, 0,0, 0.0, 0.0]

        step_lin = 0.01
        step_rot = step_lin
        cp       = start_pos
        tp       = target_pos
        last_p   = start_pos
        
        q_init = K.ArrayToJntArray([j_0[6], j_0[7], j_0[8], j_0[9], j_0[10]])

        while math.sqrt(abs(cp[0]-tp[0])**2 + abs(cp[1]-tp[1])**2 + abs(cp[2]-tp[2])**2) >= step_lin or abs(cp[3]-tp[3]) >= step_rot or abs(cp[4]-tp[4]) >= step_rot or abs(cp[5]-tp[5]) >= step_rot:
            q_s = PyKDL.JntArray(chain.getNrOfJoints())
            a = step_lin * (tp[0]-cp[0])/math.sqrt(abs(cp[0]-tp[0])**2 + abs(cp[1]-tp[1])**2 + abs(cp[2]-tp[2])**2)
            b = step_lin * (tp[1]-cp[1])/math.sqrt(abs(cp[0]-tp[0])**2 + abs(cp[1]-tp[1])**2 + abs(cp[2]-tp[2])**2)
            c = step_lin * (tp[2]-cp[2])/math.sqrt(abs(cp[0]-tp[0])**2 + abs(cp[1]-tp[1])**2 + abs(cp[2]-tp[2])**2)
            r = step_rot * (tp[3]-cp[3])/math.sqrt(abs(cp[3]-tp[3])**2 + abs(cp[4]-tp[4])**2 + abs(cp[5]-tp[5])**2)
            p = step_rot * (tp[4]-cp[4])/math.sqrt(abs(cp[3]-tp[3])**2 + abs(cp[4]-tp[4])**2 + abs(cp[5]-tp[5])**2)
            y = step_rot * (tp[5]-cp[5])/math.sqrt(abs(cp[3]-tp[3])**2 + abs(cp[4]-tp[4])**2 + abs(cp[5]-tp[5])**2)

            cp[0] = cp[0] + a
            cp[1] = cp[1] + b
            cp[2] = cp[2] + c
            cp[3] = cp[3] + r
            cp[4] = cp[4] + p
            cp[5] = cp[5] + y

            distance_lin = math.sqrt(abs(cp[0]-tp[0])**2 + abs(cp[1]-tp[1])**2 + abs(cp[2]-tp[2])**2)
            distance_rot = math.sqrt(abs(cp[3]-tp[3])**2 + abs(cp[4]-tp[4])**2 + abs(cp[5]-tp[5])**2)
            target_frame = K.PointToFrame(cp)
            ok = solver_ik.CartToJnt(q_init, target_frame, q_s)
            
            if ok < 0:
                print("Solver failed. Solver returned value: %s" % (ok))
                print q_init

                break

            print distance_lin
            print distance_rot

            last_p = cp
            q_init = q_s
            """
            tmp_point   = JointTrajectoryPoint()
            t_s = []
            q_s = []

            #                          x      y      z         dummy   dummy1  yaw     joint1        joint2        joint3        joint4        joint5
            tmp_point.positions     = [j_0[0], j_0[1], j_0[2], j_0[3], j_0[4], j_0[5], q_s[0]+j_0[6],q_s[1]+j_0[7],q_s[2]+j_0[8],q_s[3]+j_0[9],q_s[4]+j_0[10]]
            tmp_point.velocities    = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0          , 0.0         , 0.0         , 0.0         , 0.0          ]
            tmp_point.accelerations = [0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0          , 0.0         , 0.0         , 0.0         , 0.0          ]
    
            tmp_point.time_from_start.secs  = int( t_s[i])
            tmp_point.time_from_start.nsecs = int(t_s[i] *1000000000 - int(t_s[i])*1000000000)
            traj.points.append(tmp_point)
            """    
        return traj


    def GenDOFManRoutine(self, amplitudes, period, j_0):
        """ """
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



if __name__ == '__main__':
    rospy.init_node('Trajectory_Generator', anonymous=True)

    K  = Kinematics()
    TP = TrajectoryPlanning()
    frame = PyKDL.Frame.Identity()
    chain = K.GenChain()

    q_max1 = K.ArrayToJntArray([1.57, 2.2, 1.57, 1.57, 2.3])
    q_min1 = K.ArrayToJntArray([-1.57, -2.2, -1.57, -1.57, -2.3])

    solver_fk = PyKDL.ChainFkSolverPos_recursive(chain)
    solver_vel = PyKDL.ChainIkSolverVel_pinv_givens(chain)
    solver_ik = PyKDL.ChainIkSolverPos_NR_JL(chain, q_min1, q_max1, solver_fk, solver_vel)

    q_init = K.ArrayToJntArray([0.5,0.5,0.5, 0.5, 0.5])
    ok = solver_fk.JntToCart(q_init, frame, -1)
    start_point = K.FrameToPoint(frame)
    target_point = [0.0, 0, 0.74, 0, 0, 0]
    traj_to_pos = TP.GoToPoint(chain, solver_fk, solver_ik, 4.0, start_point, target_point)








