#!/usr/bin/env python

import rospy
from qcontrol_defs.msg import PVAStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32

import numpy as np
from scipy.signal import butter

class Filter(object):
    def __init__(self, b, a, length):
        coeff_len = max(len(b), len(a))
        self.b = np.zeros(coeff_len)
        self.a = np.zeros(coeff_len)
        self.b[:len(b)] = b
        self.a[:len(a)] = a

        self.x = np.zeros((coeff_len, length))
        self.y = np.zeros((coeff_len, length))

    def step(self, x):
        self.x = np.roll(self.x, 1, axis = 0)
        self.y = np.roll(self.y, 1, axis = 0)
        self.x[0] = x
        self.y[0] = np.dot(self.b, self.x) - np.dot(self.a[1:], self.y[1:])
        self.y[0] /= self.a[0]
        return self.y[0]


class ViconDerivatives(object):
    def __init__(self):
        rospy.init_node('vicon_derivatives', anonymous=True)
        self.pub_pva = rospy.Publisher('/est_pva', PVAStamped, queue_size=1)
        self.pub_dt = rospy.Publisher('/dt', Float32, queue_size=1)
        rospy.Subscriber("/vicon/mk1/mk1", TransformStamped, self.handle_position)
        self.prev_position = np.zeros(3)
        self.prev_velocity = np.zeros(3)
        self.prev_time = 0

        fs = 100
        cut_freq = 3.5
        cut = 2*cut_freq / fs
        b,a = butter(4, (cut), btype='low')
        # b = [1.0/50]*50
        # a = [1]
        self.filt = Filter(b, a, 3)

    def handle_position(self, msg):
        time = msg.header.stamp.to_sec()
        dt = time - self.prev_time
        position = np.array([msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z])
        position = self.filt.step(position)
        velocity = (position - self.prev_position)/dt
        acceleration = (velocity - self.prev_velocity)/dt

        new_msg = PVAStamped()
        new_msg.header.stamp = msg.header.stamp
        new_msg.pva.pos.position.x = position[0]
        new_msg.pva.pos.position.y = position[1]
        new_msg.pva.pos.position.z = position[2]
        new_msg.pva.vel.linear.x = velocity[0]
        new_msg.pva.vel.linear.y = velocity[1]
        new_msg.pva.vel.linear.z = velocity[2]
        new_msg.pva.acc.linear.x = acceleration[0]
        new_msg.pva.acc.linear.y = acceleration[1]
        new_msg.pva.acc.linear.z = acceleration[2]

        self.pub_pva.publish(new_msg)

        dt_msg = Float32()
        dt_msg.data = dt
        self.pub_dt.publish(dt_msg)

        self.prev_position = position
        self.prev_velocity = velocity
        self.prev_time = time
if __name__ == '__main__':
    thing = ViconDerivatives()
    rospy.spin()
