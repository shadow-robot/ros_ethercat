#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Author: Stuart Glaser

from __future__ import with_statement

import roslib
import copy
import threading
import sys, os
from time import sleep

# Loads interface with the robot.
import rospy
from std_msgs.msg import *
import controller_manager_msgs.srv as controller_manager_srvs
import controller_manager_msgs.msg as controller_manager_msgs
from std_msgs.msg import Bool

load_controller = rospy.ServiceProxy('controller_manager/load_controller', controller_manager_srvs.LoadController)
unload_controller = rospy.ServiceProxy('controller_manager/unload_controller', controller_manager_srvs.UnloadController)
switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', controller_manager_srvs.SwitchController)

def calibrate(controllers):

    success = True

    if type(controllers) is not list:
        controllers = [controllers]

    launched = []
    try:
        # Loads the controllers
        for c in controllers:
            resp = load_controller(c)
            if resp.ok == 0:
                rospy.logerr("Failed: %s" % c)
                success = False
            else:
                launched.append(c)
        print "Launched: %s" % ', '.join(launched)

        # Starts the launched controllers
        switch_controller(launched, [], controller_manager_srvs.SwitchControllerRequest.BEST_EFFORT)

        # Sets up callbacks for calibration completion
        waiting_for = launched[:]
        def calibrated(msg, name):  # Somewhat not thread-safe
            if name in waiting_for:
                waiting_for.remove(name)
        for name in waiting_for:
            rospy.Subscriber("%s/calibrated" % name, Empty, calibrated, name)

        # Waits until all the controllers have calibrated
        while waiting_for and not rospy.is_shutdown():
            print "Waiting for: %s" % ', '.join(waiting_for)
            sleep(0.5)
    finally:
        for name in launched:
            try:
                resp_stop = switch_controller([], [name], controller_manager_srvs.SwitchControllerRequest.STRICT)
                if (resp_stop == 0):
                    rospy.logerr("Failed to stop controller %s" % name)
                resp_unload = unload_controller(name)
                if (resp_unload == 0):
                    rospy.logerr("Failed to unload controller %s" % name)
            except Exception, ex:
                rospy.logerr("Failed to stop/unload controller %s" % name)
    return success

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
            self.sub = rospy.Subscriber(topic,Bool,self.callback)
            try:
                with self.cond:
                    if not self.is_calibrated:
                        self.cond.wait(timeout)
                return self.is_calibrated
            finally:
                self.sub.unregister()
            return self.is_calibrated

    print "Waiting up to 20s for IMU calibration to complete."
    helper = is_calibrated_helper()
    if not helper.wait_for_calibrated("torso_lift_imu/is_calibrated", 20):
        rospy.logerr("IMU took too long to calibrate.")
        return False
    return True

def main():
    pub_calibrated = rospy.Publisher('calibrated', Bool, latch=True)
    rospy.wait_for_service('controller_manager/load_controller')
    rospy.wait_for_service('controller_manager/switch_controller')
    rospy.wait_for_service('controller_manager/unload_controller')
    if  rospy.is_shutdown(): return

    rospy.init_node('calibration', anonymous=True)
    pub_calibrated.publish(False)

    # Don't calibrate the IMU unless ordered to by user
    cal_imu = rospy.get_param('calibrate_imu', False)

    if cal_imu:
        imustatus = calibrate_imu()
    else: 
        imustatus = True

    xml = ''

    controllers = rospy.myargv()[1:]

    if not calibrate(controllers):
        sys.exit(3)

    pub_calibrated.publish(True)

    if not imustatus:
        print "Mechanism calibration complete, but IMU calibration failed."
    else:
        print "Calibration complete"

    rospy.spin()

if __name__ == '__main__':
    main()
