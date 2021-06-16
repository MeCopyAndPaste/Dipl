#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from geometry_msgs.msg import Pose
import math
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose                                                              #
from geometry_msgs.msg import Vector3                                                           #
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from PyKDL import Frame

class Position:
    x = None 
    y = None 
    z = None 
    roll  = None
    pitch = None
    yaw = None


    def Test1(self):
        start_pos = Position()
        start_pos.x = 0
        start_pos.y = 0
        start_pos.z = 1
        start_pos.roll = 0
        start_pos.pitch = 0
        start_pos.yaw = 0

        A = 0.5
        f = 1/8

        pitches = []
        yaws    = []


        for i in range(60):
            pitches.append(A*(1-math.cos(i*2*pi*f))*pi/2)
            yaws.append(A*(1-math.cos(i*2*pi*f))*pi/2)
        
    def GenQuasiSinVelTrajOnAxis(traj, axis, amplitude, period, phase_start, phase_end , points_no):
        T           = float(period)                                                                                             # casting in to avoid possible error that could be caused
        A           = float(amplitude)                                                                                          # by forwarding the wrong data type to function
        phase_start = float(phase_start)                                                                                        #
        phase_end   = float(phase_end)                                                                                          #
        points_no   = int(points_no)                                                                                            #
        w           = 2*math.pi*(1/T)                                                                                           #
        t_s         = np.linspace(phase_start/(2.0*math.pi)*T, phase_end/(2.0*math.pi)*T, points_no)                            #
        v_s         = []
        s_s         = []

        for t in t_s:
            v_s.append( A * math.sin(w*t + phase_start) )
            s_s.append(-A/w * math.cos(w*t + phase_start)+ A/w)

        plt.figure()
        plt.plot(t_s, v_s, 'b',t_s, s_s,'g')
        plt.show()








    def DHToFrame(self, a, alpha, d, theta):
        M1 = PyKDL.Vector( math.cos(theta), -1.0*math.cos(alpha)*math.sin(theta),      math.sin(alpha)*math.sin(theta) )
        M2 = PyKDL.Vector( math.sin(theta),      math.cos(alpha)*math.cos(theta), -1.0*math.sin(alpha)*math.cos(theta) )
        M3 = PyKDL.Vector(              0,                       math.sin(alpha),                      math.cos(alpha) )
        M  = PyKDL.Rotation(x = M1, y = M2, z = M3)
        p  = PyKDL.Vector( a*math.cos(theta), a*math.sin(theta), d)

        return PyKDL.Frame(M, p)



    def GenChain(self):

        chain = PyKDL.Chain()

        f = self.DHToFrame(0.0, -1.0*math.pi, 0.0353, 0.0)
        segment_tmp = PyKDL.Segment(name = u"base", joint = PyKDL.Joint(PyKDL.Joint.None), f_tip = self.DHToFrame(a = 0.0, alpha = -0.5*math.pi, d = 0.0353, theta = 0.0))        
        chain.addSegment(segment_tmp)

        joint_tmp   = PyKDL.Joint(name = "joint1",  type = PyKDL.Joint.RotZ)
        segment_tmp = PyKDL.Segment(name = u"segment1", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.1225, alpha = 0.0, d = 0.0, theta = -0.5*math.pi))        
        chain.addSegment(segment_tmp)
        
        joint_tmp   = PyKDL.Joint(name = "joint2",  type = PyKDL.Joint.RotZ)
        segment_tmp = PyKDL.Segment(name = u"segment2", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.1365, alpha = -0.5*math.pi, d = 0.0, theta = 0.0 ))        
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
        segment_tmp = PyKDL.Segment(name = u"virtual1", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.0, alpha = -0.5*math.pi, d = 0.0, theta = -0.5*math.pi))        
        chain.addSegment(segment_tmp)

        joint_tmp   = PyKDL.Joint(name = "tool_fixed_jnt",  type = PyKDL.Joint.Fixed)
        segment_tmp = PyKDL.Segment(name = u"tool", joint = joint_tmp, f_tip = self.DHToFrame(a = 0.0, alpha = 0.0, d = 0.32, theta = -0.5*math.pi))        
        chain.addSegment(segment_tmp)

        return chain


    def TheIK(self):

        chain   = self.GenChain()
        solver  = PyKDL.ChainFkSolverPos_recursive(chain)
        solver2 = PyKDL.ChainIkSolverVel_pinv_givens(chain)
        q_max   = PyKDL.JntArray(chain.getNrOfJoints())
        q_min   = PyKDL.JntArray(chain.getNrOfJoints())
        solverik= PyKDL.ChainIkSolverPos_NR_JL(chain, q_min, q_max, solver, solver2)
        jnt_arr = PyKDL.JntArray(chain.getNrOfJoints())
        jnt_arr2= PyKDL.JntArray(chain.getNrOfJoints())

        fk_res_frame = PyKDL.Frame()
        Chk = solver.JntToCart(jnt_arr, fk_res_frame, 8)
        Ch2 = solverik.CartToJnt(jnt_arr, fk_res_frame, jnt_arr2)

        print("jnt_arr")
        print(jnt_arr)
        print("\nframe")
        print(fk_res_frame)
        print("\njnt_arr2")
        print(jnt_arr2)
        print("\nCh2")
        print(Ch2)
        #out = PyKDL.ChainFkSolverPos()


        #print(chain.getNrOfSegments())



    def TimeVelRefTest(self):

        pub_vel_ref = rospy.Publisher('/uav/vel_ref', Vector3, queue_size=10)
        #rospy.init_node('TimeVelRefTest', anonymous=True)

        start_time = rospy.get_time()
        now_time = start_time


        time = []
        vel  = []

        while now_time-start_time < 8.0:
            ref = Vector3()
            delta_t = now_time-start_time
            A = 0.5
            T = 8 
            now_time = rospy.get_time()       
            time.append(now_time-start_time)

            ref.x = 0.0
            ref.y = 0.0
            ref.z = 0.25 * math.sin(2* math.pi/T * (now_time-start_time))
            pub_vel_ref.publish(ref)

            vel.append(ref.z)
            rospy.sleep(0.1)


        plt.plot(time, vel)
        plt.show()


if __name__ == '__main__':
    TEST = Position()
    #TEST.TimeVelRefTest()
    TEST.TheIK()
    #GenQuasiSinVelTrajOnAxis(JointTrajectory(), 'x', 0.5, 8, 0, 2*math.pi, 1000)