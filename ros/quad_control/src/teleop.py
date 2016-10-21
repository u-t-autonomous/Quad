#!/usr/bin/env python

import rospy
from qcontrol_defs.msg import PVAStamped
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from vicon_bridge.srv import viconGrabPose
from std_srvs.srv import Trigger, SetBool

class Posing():
    def __init__(self):
        rospy.init_node('publish_pose', anonymous=True)
        self.dt = 1.0/10.0
        self.max_hvel = 2.0
        self.max_vvel = 0.5
        self.max_hdg_rate = 2.0*3.14159/3.0
        self.max_acc = 9.8*0.5
        self.joy = Joy()
        self.joy.axes = [0]*8
        self.joy.buttons = [0]*11
        self.prev_joy = self.joy
        self.current_position = None

        # Wait for MikiPilot services
        rospy.wait_for_service('/set_armed')
        self.set_armed = rospy.ServiceProxy('/set_armed', SetBool)
        rospy.wait_for_service('/standby')
        self.standby = rospy.ServiceProxy('/standby', Trigger)
        rospy.wait_for_service('/mp_hold_motor')
        self.mp_hold_motor = rospy.ServiceProxy('/mp_hold_motor', Trigger)
        rospy.wait_for_service('/mp_hold_attitude')
        self.mp_hold_attitude = rospy.ServiceProxy('/mp_hold_attitude', Trigger)
        rospy.wait_for_service('/mp_hold_velocity')
        self.mp_hold_velocity = rospy.ServiceProxy('/mp_hold_velocity', Trigger)
        rospy.wait_for_service('/shim_pva')
        self.shim_pva = rospy.ServiceProxy('/shim_pva', Trigger)
        rospy.wait_for_service('/pva_control')
        self.pva_control = rospy.ServiceProxy('/pva_control', Trigger)

        rospy.wait_for_service('/vicon/grab_vicon_pose')
        self.get_current_pose = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)

        self._pub_pva = rospy.Publisher('/pva', PVAStamped, queue_size=1)

        rospy.Subscriber("/joy", Joy, self.joyChanged)
        rospy.Subscriber("/vicon/mk1/pose", PoseStamped, self.positionChanged)

        # rate = rospy.Rate(1.0)
        # while not rospy.is_shutdown():
        #     if(self.current_position is not None):
        #         break
        #     rate.sleep()

        self.stay()

    def stay(self):
        self._msg_pva = PVAStamped()
        self._msg_pva.header.stamp = rospy.Time.now()
        resp = self.get_current_pose("mk1", "mk1", 10)
        self._msg_pva.pva.pos.position = resp.pose.pose.position
        # self._msg_pva.pos.position = self.current_position

    def positionChanged(self, msg):
        self.current_position = msg.pose.position

    def joyChanged(self, msg):
        self.joy = msg

    def updateAxes(self):

        self._msg_pva.pva.acc.linear.x = -self.joy.axes[4]*self.max_acc
        self._msg_pva.pva.acc.linear.y = -self.joy.axes[3]*self.max_acc
        self._msg_pva.pva.acc.linear.z = self.joy.axes[1]*self.max_acc

        self._msg_pva.pva.vel.linear.x += self._msg_pva.pva.acc.linear.x*self.dt
        self._msg_pva.pva.vel.linear.y += self._msg_pva.pva.acc.linear.y*self.dt
        self._msg_pva.pva.vel.linear.z += self._msg_pva.pva.acc.linear.z*self.dt
        # self._msg_pva.vel.angular.z = self.joy.axes[1]*self.max_hdg_rate

        self._msg_pva.pva.pos.position.x += self._msg_pva.pva.vel.linear.x*self.dt
        self._msg_pva.pva.pos.position.y += self._msg_pva.pva.vel.linear.y*self.dt
        self._msg_pva.pva.pos.position.z += self._msg_pva.pva.vel.linear.z*self.dt
        # TODO: Integrate angular z velocity to get heading and convert to quaternion

        if self.joy.buttons[4] == 1 or self.joy.buttons[5] == 1:
            self.stay()
        if self.joy.buttons[1] == 1 and self.prev_joy.buttons[1] == 0:
            self.standby()
            self.set_armed(False)
        if self.joy.buttons[0] == 1 and self.prev_joy.buttons[0] == 0:
            self.mp_hold_velocity()
        if self.joy.buttons[6] == 1 and self.joy.buttons[7] == 1:
            self.set_armed(True)
        if self.joy.buttons[2] == 1 and self.prev_joy.buttons[2] == 0:
            self.mp_hold_motor()
        if self.joy.buttons[3] == 1 and self.prev_joy.buttons[3] == 0:
            self.stay()
            self.pva_control()
            # self.mp_hold_attitude()

        self._msg_pva.header.stamp = rospy.Time.now()
        self._pub_pva.publish(self._msg_pva)

        self.prev_joy = self.joy

if __name__ == '__main__':
    posing = Posing()

    rate = rospy.Rate(1.0/posing.dt)

    while not rospy.is_shutdown():
        posing.updateAxes()
        rate.sleep()
