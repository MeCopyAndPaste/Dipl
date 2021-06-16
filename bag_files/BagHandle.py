#!/usr/bin/env python
import rosbag
import subprocess, shlex, psutil
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler             # Modules needed for data processing


class BagHandle:

    def __init__(self):
        pass

    def _StartBagRecordAll(self):
        StartBagRecordAll()

    def _StopBagRecordAll(self, rosbag_proc):
        StopBagRecordAll()

    def _PlotRPY(self):
        PlotRPY()
        
def StartBagRecordAll():

    command = "rosbag record -O /Bag_files/all_topics.bag -a"
    command = shlex.split(command)
    rosbag_proc = subprocess.Popen(command)

    return rosbag_proc

def StopBagRecordAll(rosbag_proc):
    for proc in psutil.process_iter():
        if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
            proc.send_signal(subprocess.signal.SIGINT)

    rosbag_proc.send_signal(subprocess.signal.SIGINT)


def PlotRPY():
    bag = rosbag.Bag('repaired.bag')
    
    rll_ref = []
    ptc_ref = []
    yaw_ref = []

    rll = []
    ptc = []
    yaw = []

    Z = []

    topic = '/uav/euler_ref'
    for topic, msg, t in bag.read_messages(topics=['/uav/euler_ref']):
        rll_ref.append(msg.x)
        ptc_ref.append(msg.y)
        yaw_ref.append(msg.z)

    topic = '/gazebo/link_states'
    for topic, msg, t in bag.read_messages(topics=['/gazebo/link_states']):
        idx = msg.name.index('uav::base_link') 
        quat_or   = msg.pose[idx].orientation
        Z.append(msg.pose[idx].position.z)
        (t1, t2, t3) = euler_from_quaternion([quat_or.x, quat_or.y, quat_or.z, quat_or.w])
        rll.append(t1)
        ptc.append(t2)
        yaw.append(t3)
    
    print(Z)
    bag.close()

    print(len(rll_ref))
    print(len(rll))

    plt.figure(1)
    plt.subplot(411)
    plt.title('roll')
    plt.plot(range(len(rll)), rll, 'r')
    plt.subplot(412)
    plt.title('pitch')
    plt.plot(range(len(ptc)), ptc, 'r')
    plt.subplot(413)
    plt.title('yaw')
    plt.plot(range(len(yaw)), yaw, 'r')
    plt.subplot(414)
    plt.title('height')
    plt.plot(range(len(Z)), Z, 'r')


    plt.figure(2)
    plt.subplot(311)
    plt.title('roll ref')
    plt.plot(range(len(rll_ref)), rll_ref, 'b')
    plt.subplot(312)
    plt.title('pitch ref')
    plt.plot(range(len(ptc_ref)), ptc_ref, 'b')
    plt.subplot(313)
    plt.title('yaw ref')
    plt.plot(range(len(yaw_ref)), yaw_ref, 'b')
    

    plt.show()


if __name__ == '__main__':
    PlotRPY()
    

