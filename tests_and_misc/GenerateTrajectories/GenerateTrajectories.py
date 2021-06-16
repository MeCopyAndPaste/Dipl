#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from geometry_msgs.msg import Pose
import math
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose                                                              
from geometry_msgs.msg import Vector3                                                           
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from PyKDL import Frame

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



class Kinematics:
    
    def __init__(self):
        pass

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

        tmp_1 = PyKDL.Rotation()
        tmp_2 = PyKDL.Rotation()
        tmp_3 = PyKDL.Rotation()
        tmp_M = PyKDL.Rotation()
        tmp_1.DoRotX(target[3])
        tmp_2.DoRotY(target[4])
        tmp_3.DoRotZ(target[5])
        tmp_M = tmp_3 * tmp_2 * tmp_1
        tmp_p = PyKDL.Vector(target[0], target[1], target[2])  
        
        return PyKDL.Frame(tmp_M, tmp_p)

    def GenChain(self):
        """Creates a KDL kinematic chain. Chain represents the manipulator mounted on the quadrotor drone. Chain is generated using hardoded DH parameters"""
        
        chain = PyKDL.Chain()

        f = self.DHToFrame(0.0, -1.0*math.pi, 0.0353, 0.0)
        segment_tmp = PyKDL.Segment(name = u"base", joint = PyKDL.Joint(PyKDL.Joint.None), f_tip = self.DHToFrame(a = 0.0, alpha = -0.5*math.pi, d = 0.0353, theta = 0.0))        
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


    def Test(self):

        #robot from DH parameters:
        chain   = self.GenChain()
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
        q_min[4] =-2.3
       
        ############################################################################################################################################################
        #creating solvers:
        solver_fk  = PyKDL.ChainFkSolverPos_recursive(chain)
        solver_vel = PyKDL.ChainIkSolverVel_pinv_givens(chain)
        solver_ik  = PyKDL.ChainIkSolverPos_NR_JL(chain, q_min, q_max, solver_fk, solver_vel)
        ############################################################################################################################################################

        #target = self.PointToFrame()


        fk_res_frame = PyKDL.Frame()

        
        jnt_arr  = PyKDL.JntArray(chain.getNrOfJoints())
        jnt_arr2 = PyKDL.JntArray(chain.getNrOfJoints())


                
        Ch1 = solver_fk.JntToCart(jnt_arr, fk_res_frame, -1)
        Ch2 = solver_ik.CartToJnt(jnt_arr, fk_res_frame, jnt_arr2)

    def Forward(self, joints):
        """Description not implemented"""

        tool_frame = PyKDL.Frame
        chain      = self.GenChain()
        jnt_arr    = PyKDL.JntArray(chain.getNrOfJoints())
        jnt_arr[0] = joints[0]
        jnt_arr[1] = joints[1]
        jnt_arr[2] = joints[2]
        jnt_arr[3] = joints[3]
        jnt_arr[4] = joints[4]
        solver_fk  = PyKDL.ChainFkSolverPos_recursive(chain)
        
        ok = solver_fk.JntToCart(jnt_arr, tool_frame, -1)
        tool_xyt_rpy = self.PointToFrame(tool_frame)

        return [ok, tool_xyt_rpy]

    def Inverse(self, tool_xyt_rpy, current_joint_positions):
        """Description not implemented"""

        #robot from DH parameters:
        chain   = self.GenChain()
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
        q_min[4] =-2.3       
        ############################################################################################################################################################
        #creating solvers:
        solver_fk  = PyKDL.ChainFkSolverPos_recursive(chain)
        solver_vel = PyKDL.ChainIkSolverVel_pinv_givens(chain)
        solver_ik  = PyKDL.ChainIkSolverPos_NR_JL(chain, q_min, q_max, solver_fk, solver_vel)
        ############################################################################################################################################################
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
        ok = solver_ik.CartToJnt(jnt_start_pos, fk_res_frame, jnt_end_pos)
        ############################################################################################################################################################
        #converting joint lists into PyKDL joint arrays:
        joints = []
        joints.append(jnt_end_pos[0])
        joints.append(jnt_end_pos[1])
        joints.append(jnt_end_pos[2])
        joints.append(jnt_end_pos[3])
        joints.append(jnt_end_pos[4])

        return [ok, joints]



class TrajectoryPlanning:

    def __init__(self):
        pass

    def GenSinXYZRoutineTraj(self, amplitude, period, Hz):
        """Generates a JointTrajectoryPoint message for UAV motion in acquisition routine. 
           UAV motion involves motion along: X-axis, Y-axis, Z-axis (height above ground), and yaw motion"""

        traj                 = JointTrajectory()
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()

        T           = float(period/2)                                                           # casting to avoid possible error that could be 
        A           = float(amplitude)                                                          # caused by passing the wrong data type to function
        w           = 2*math.pi*(1/T)                                                           #        
        small_pause = 2.0                                                                       # Pause between fowards and backwards movements on same degree of freedom
        big_pause   = 2.0                                                                       # Pause between movements ivolving different degrees of freedom
        points_no   = int((T + small_pause + big_pause)*Hz)                                     #
        t_s         = np.linspace(0, period+small_pause+big_pause, points_no)                   #

        last_t = t_s[-1]
        count  = 0.0

        for axis in ['x', 'y', 'z', 'yaw']:

            for i in range(len(t_s)):
                tmp_point   = JointTrajectoryPoint()

                if (t_s[i] >= T/2.0 and t_s[i] <= (T/2.0 + small_pause)):                       # Setting values for the points of desired trajectory
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
                    v_s = A   * math.sin( w*(t_s[i] - 2.0))                                     #  
                    s_s =-A/w * math.cos( w*(t_s[i] - 2.0)) + A/w                               # 
                    a_s = A*w * math.cos( w*(t_s[i] - 2.0))                                     #
 
                if axis == 'x' or axis == 'X':
                    #                          x         y         z         dummy  dummy1  yaw         joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [s_s+x_0 , y_0     , z_0     , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                    tmp_point.velocities    = [v_s       , 0.0   , 0.0     , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                    tmp_point.accelerations = [a_s       , 0.0   , 0.0     , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                elif axis == 'y' or axis == 'Y':
                    #                          x         y         z         dummy  dummy1  yaw         joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [x_0     , s_s+y_0 , z_0     , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                    tmp_point.velocities    = [0.0     , v_s     , 0.0     , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                    tmp_point.accelerations = [0.0     , a_s     , 0.0     , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]

                elif axis == 'z' or axis == 'Z':
                    #                          x         y         z         dummy  dummy1  yaw         joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [0.0     , 0.0     , s_s+z_0 , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                    tmp_point.velocities    = [0.0     , 0.0     , v_s     , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                    tmp_point.accelerations = [0.0     , 0.0     , a_s     , 0.0  , 0.0   , 0.0       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]

                elif axis == 'yaw' or axis == 'yaw':
                    #                          x         y         z         dummy  dummy1  yaw         joint1  joint2  joint3  joint4  joint5  
                    tmp_point.positions     = [x_0     , 0.0     , z_0     , 0.0  , 0.0   , s_s+yaw_0 , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                    tmp_point.velocities    = [0.0     , 0.0     , 0.0     , 0.0  , 0.0   , v_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
                    tmp_point.accelerations = [0.0     , 0.0     , 0.0     , 0.0  , 0.0   , a_s       , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ]
            
                tmp_point.time_from_start.secs  = int( t_s[i] + last_t * count)
                tmp_point.time_from_start.nsecs = int((t_s[i] + last_t * count)*1000000000 - int(t_s[i] + last_t * count)*1000000000)
                traj.points.append(tmp_point)

            count = count + 1.0                                                                

        return traj


    def GenJointsJoint0ManRoutine(self):
        """ """

        traj                 = JointTrajectory()
        traj.header.frame_id = 'base_link'
        traj.joint_names     = ['x','y','z','dummy','dummy1','yaw','joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp    = rospy.get_rostime()




if __name__ == '__main__':
    TEST = Kinematics()

    TEST.Test()


