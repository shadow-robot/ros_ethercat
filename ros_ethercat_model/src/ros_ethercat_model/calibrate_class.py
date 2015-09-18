#!/usr/bin/env python
#
# Copyright 2014 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.


import threading
from time import sleep

# Loads interface with the robot.
import rospy
import controller_manager_msgs.srv as controller_manager_srvs
from std_msgs.msg import Bool, Empty


class Calibrate(object):

    def __init__(self):
        rospy.loginfo("waiting for the controller manager")
        rospy.wait_for_service('controller_manager/load_controller', timeout=40.0)
        rospy.wait_for_service('controller_manager/unload_controller', timeout=40.0)
        rospy.wait_for_service('controller_manager/switch_controller', timeout=40.0)
        rospy.loginfo("OK the controller manager is ready")

        self.load_controller = rospy.ServiceProxy('controller_manager/load_controller',
                                                  controller_manager_srvs.LoadController)
        self.unload_controller = rospy.ServiceProxy('controller_manager/unload_controller',
                                                    controller_manager_srvs.UnloadController)
        self.switch_controller = rospy.ServiceProxy('controller_manager/switch_controller',
                                                    controller_manager_srvs.SwitchController)
        self.pub_calibrated = rospy.Publisher('calibrated', Bool, queue_size=1, latch=True)

    def calibrate(self, controllers):

        success = True

        if type(controllers) is not list:
            controllers = [controllers]

        launched = []
        try:
            # Loads the controllers
            for c in controllers:
                resp = self.load_controller(c)
                if resp.ok == 0:
                    rospy.logerr("Failed: %s" % c)
                    success = False
                else:
                    launched.append(c)
            print("Launched: %s" % ', '.join(launched))

            # Starts the launched controllers
            self.switch_controller(launched, [], controller_manager_srvs.SwitchControllerRequest.BEST_EFFORT)

            # Sets up callbacks for calibration completion
            waiting_for = launched[:]

            def calibrated(msg, name):  # Somewhat not thread-safe
                if name in waiting_for:
                    waiting_for.remove(name)
            for name in launched:
                rospy.Subscriber("%s/calibrated" % name, Empty, calibrated, name)

            # Waits until all the controllers have calibrated
            while waiting_for and not rospy.is_shutdown():
                print("Waiting for: %s" % ', '.join(waiting_for))
                sleep(0.5)
        finally:
            for name in launched:
                try:
                    resp_stop = self.switch_controller([], [name], controller_manager_srvs.SwitchControllerRequest.STRICT)
                    if (resp_stop == 0):
                        rospy.logerr("Failed to stop controller %s" % name)
                    resp_unload = self.unload_controller(name)
                    if (resp_unload == 0):
                        rospy.logerr("Failed to unload controller %s" % name)
                except Exception, ex:
                    rospy.logerr("Failed to stop/unload controller %s" % name)
        return success

    @staticmethod
    def calibrate_imu():
        class is_calibrated_helper:
            def __init__(self):
                self.is_calibrated = False
                self.cond = threading.Condition()

            def callback(self, msg):
                if msg.data:
                    with self.cond:
                        self.is_calibrated = True
                        self.cond.notify()

            def wait_for_calibrated(self, topic, timeout):
                self.sub = rospy.Subscriber(topic, Bool, self.callback)
                try:
                    with self.cond:
                        if not self.is_calibrated:
                            self.cond.wait(timeout)
                    return self.is_calibrated
                finally:
                    self.sub.unregister()
                return self.is_calibrated

        print("Waiting up to 20s for IMU calibration to complete")
        helper = is_calibrated_helper()
        if not helper.wait_for_calibrated("torso_lift_imu/is_calibrated", 20):
            rospy.logerr("IMU took too long to calibrate.")
            return False
        return True
