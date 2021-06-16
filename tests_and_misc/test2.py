#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import math

rospy.init_node('test2', anonymous=True)

start_time = rospy.get_time()
now_time = start_time


time = []
ref  = []

while now_time-start_time < 8.0:
    delta_t = now_time-start_time
    A = 0.5
    T = 8 
    now_time = rospy.get_time()       
    time.append(now_time-start_time)
    ref.append(1.0/16.0 * math.sin(2* math.pi/T * (now_time-start_time)))

plt.plot(time, ref)
plt.show()