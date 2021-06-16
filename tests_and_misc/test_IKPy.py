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
from ikpy import chain as IkPyChain                                                             # pip install sympy==0.7.2
from ikpy.link import OriginLink, URDFLink
import kdl_parser_py
from mpl_toolkits.mplot3d import Axes3D



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

    def TheIK(self):
        print(0.04526+ 0.122490+ 0.1365+ 0.075511 + 0.072489+ 0.04526 + 0.29)

        chain = self.ReturnChain()
        #ax = plt.figure().add_subplot(111, projection='3d')
        #chain.plot(chain.inverse_kinematics(target_position=[0.3, 0, 0.4],target_orientation=[0,0,0]), ax)
        #plt.show()
        sol = [0, 0, 0, 0, 0, 0, 0, 0]
        print(sol)
        transf = chain.forward_kinematics(sol)
        #chain.plot( chain.forward_kinematics(sol),ax)
        print(transf)
    def ReturnChain(self):
        a0 = 0.04526
        a1 = 0.122490
        a2 = 0.1365
        a3 = 0.075511
        a4 = 0.072489
        d5 = 0.04526
        theta1 = math.pi/2.0
        alpha2 =-math.pi/2.0
        tool_length = 0.3

        Kin_Chain = IkPyChain.Chain(name='aerial_manipulator', links = [
            OriginLink(),
            URDFLink(
                name="dummy_base_joint",
                translation_vector=[0, 0, 0],
                orientation       =[0, 0, 0],
                rotation          =[0, 0, 0],               #joint_axis
            ),
            URDFLink(
                name="joint1",
                translation_vector=[0, 0, a0],
                orientation       =[1.57079632679, 3.14159265359, 0],
                rotation          =[0, 0, 1],               #joint_axis
            ),
            URDFLink(
                name="joint2",
                translation_vector=[0, a1, 0],
                orientation       =[0, 0, 1.57079632679],
                rotation          =[0, 0, 1],               #joint frame axis of rotation
            ),   
            URDFLink(
                name="joint3",
                translation_vector=[a2, 0, 0],
                orientation       =[-1.57079632679, 0, 0],
                rotation          =[0, 0, 1],               #joint frame axis of rotation
            ),
            URDFLink(
                name="joint5",
                translation_vector=[a3, 0, 0],
                orientation       =[0, 0, 0],
                rotation          =[1, 0, 0],               #joint frame axis of rotation
            ),
            URDFLink(
                name="joint5",
                translation_vector=[a4, 0, 0],
                orientation       =[0, 0, 0],
                rotation          =[0, 0, 1],               #joint frame axis of rotation
            ),
            
            URDFLink(
                name="dummy_tool_joint",
                translation_vector=[0, 0, d5+tool_length],
                orientation       =[0, 0, 0],
                rotation          =[0, 0, 0],               #joint frame axis of rotation
            ),
        ], active_links = [
            URDFLink(
                name="joint1",
                translation_vector=[0, 0, a0],
                orientation       =[1.57079632679, 3.14159265359, 0],
                rotation          =[0, 0, 1],               #joint_axis
            ),
            URDFLink(
                name="joint2",
                translation_vector=[0, a1, 0],
                orientation       =[0, 0, 1.57079632679],
                rotation          =[0, 0, 1],               #joint frame axis of rotation
            ),   
            URDFLink(
                name="joint3",
                translation_vector=[a2, 0, 0],
                orientation       =[-1.57079632679, 0, 0],
                rotation          =[0, 0, 1],               #joint frame axis of rotation
            ),
            URDFLink(
                name="joint5",
                translation_vector=[a3, 0, 0],
                orientation       =[0, 0, 0],
                rotation          =[1, 0, 0],               #joint frame axis of rotation
            ),
            URDFLink(
                name="joint5",
                translation_vector=[a4, 0, 0],
                orientation       =[0, 0, 0],
                rotation          =[0, 0, 1],               #joint frame axis of rotation
            ),])

        #Kin_Chain = IkPyChain.Chain.from_urdf_file("model.urdf")



        return Kin_Chain

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