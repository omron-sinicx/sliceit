#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X
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
#  * Neither the name of OMRON SINIC X nor the names of its
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
# Author: Cristian C. Beltran-Hernandez, Felix von Drigalski

import actionlib
import rospy
from aist_camera_multiplexer import RealSenseMultiplexerClient
import o2ac_msgs.msg
import std_srvs.srv
import std_msgs.msg
import geometry_msgs.msg

from o2ac_routines.helpers import check_for_real_robot, check_for_real_robot_or_gazebo  # , lock_vision
import threading


class VisionClient():
    def __init__(self):
        self.use_real_robot = rospy.get_param("use_real_robot", False)
        self.use_gazebo_sim = rospy.get_param("use_gazebo_sim", False)

        if self.use_real_robot:
            try:
                self.vision_multiplexer = RealSenseMultiplexerClient('camera_multiplexer')
            except:
                self.vision_multiplexer = []

        self.ssd_client = actionlib.SimpleActionClient('/o2ac_vision_server/get_3d_poses_from_ssd', o2ac_msgs.msg.get3DPosesFromSSDAction)
        self.detect_shaft_client = actionlib.SimpleActionClient('/o2ac_vision_server/detect_shaft_hole', o2ac_msgs.msg.shaftHoleDetectionAction)
        self.detect_angle_client = actionlib.SimpleActionClient('/o2ac_vision_server/detect_angle', o2ac_msgs.msg.detectAngleAction)
        self.pick_success_client = actionlib.SimpleActionClient('/o2ac_vision_server/check_pick_success', o2ac_msgs.msg.checkPickSuccessAction)
        self.localization_client = actionlib.SimpleActionClient('/o2ac_vision_server/localize_object', o2ac_msgs.msg.localizeObjectAction)
        self.pulley_screw_detection_stream_client = rospy.ServiceProxy('/o2ac_vision_server/activate_pulley_screw_detection', std_srvs.srv.SetBool)
        self.pulley_screw_detection_streaming = False

        self.vision_lock = threading.Lock()

    @check_for_real_robot
    # @lock_vision
    def activate_camera(self, camera_name="b_bot_outside_camera"):
        try:
            if self.vision_multiplexer:
                return self.vision_multiplexer.activate_camera(camera_name)
            else:
                rospy.logwarn("Camera multiplexer not functional! Returning true")
                return True
        except Exception as e:
            print("Exception in activate_camera: ", e)
            pass
        rospy.logwarn("Could not activate camera! Returning false")
        return False

    @check_for_real_robot_or_gazebo
    # @lock_vision
    def read_from_ssd(self):
        """
        Returns object poses as estimated by the SSD neural network and reprojection.
        Also updates self.objects_in_tray
        """
        # Send goal, wait for result
        self.ssd_client.send_goal(o2ac_msgs.msg.get3DPosesFromSSDGoal())
        if (not self.ssd_client.wait_for_result(rospy.Duration(15.0))):
            self.ssd_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call for SSD result returned no result. Is o2ac_vision running?")
            return False

        # Read result and return
        try:
            return self.ssd_client.get_result().results
        except Exception as e:
            rospy.logerr("Exception at get_3d_poses_from_ssd %s" % e)
        return False

    @check_for_real_robot
    # @lock_vision
    def get_angle_from_vision(self, camera="b_bot_inside_camera", item_name="bearing"):
        # Send goal, wait for result
        goal = o2ac_msgs.msg.detectAngleGoal()
        goal.item_id = item_name
        self.detect_angle_client.send_goal(goal)
        if (not self.detect_angle_client.wait_for_result(rospy.Duration(3.0))):
            self.detect_angle_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call to detect angle returned no result. Is o2ac_vision running?")
            return False

        # Read result and return
        try:
            res = self.detect_angle_client.get_result()
            if res.succeeded:
                return res.rotation_angle
        except:
            pass
        return False

    @check_for_real_robot
    # @lock_vision
    def get_motor_angle_from_top_view(self, camera="b_bot_outside_camera"):
        # Send goal, wait for result
        goal = o2ac_msgs.msg.detectAngleGoal()
        goal.item_id = "motor"  # Unused
        goal.get_motor_from_top = True
        self.detect_angle_client.send_goal(goal)
        if (not self.detect_angle_client.wait_for_result(rospy.Duration(3.0))):
            self.detect_angle_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call to detect angle returned no result. Is o2ac_vision running?")
            return None

        # Read result and return
        try:
            res = self.detect_angle_client.get_result()
            if res.succeeded:
                return res.rotation_angle
        except:
            pass
        return None

    @check_for_real_robot
    # @lock_vision
    def call_shaft_hole_detection(self):
        """
        Calls the action and returns the result as is
        """
        goal = o2ac_msgs.msg.shaftNotchDetectionGoal()
        self.detect_shaft_client.send_goal(goal)
        if (not self.detect_shaft_client.wait_for_result(rospy.Duration(3.0))):
            self.detect_shaft_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call to shaft detection returned no result. Is o2ac_vision running?")
            return False
        res = self.detect_shaft_client.get_result()
        return res

    # @lock_vision
    def activate_pulley_screw_detection(self, activate=True):
        # Activate screw detection stream
        req = std_srvs.srv.SetBoolRequest()
        req.data = activate
        self.pulley_screw_detection_stream_client.call(req)
        self.pulley_screw_detection_streaming = activate

    # @lock_vision
    def check_pick_success(self, object_name):
        """ Returns true if the visual pick success check for the object returns True.
            This can only be used in specific pre-determined situations and for certain items.
            See the evaluatePickSuccess.action file.
        """
        goal = o2ac_msgs.msg.checkPickSuccessGoal()
        goal.item_id = object_name
        self.pick_success_client.send_goal(goal)
        if (not self.pick_success_client.wait_for_result(rospy.Duration(3.0))):
            self.pick_success_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call to pick success returned no result. Is o2ac_vision running?")
            return False
        res = self.pick_success_client.get_result()
        return res.item_is_picked

    # @lock_vision
    def localize_object(self, object_type):
        """
        Returns object pose if object was detected in current camera view,
        False otherwise.

        item_type is the name of the mesh file in o2ac_parts_description.
        """
        self.localization_client.send_goal(o2ac_msgs.msg.localizeObjectGoal(item_id=object_type))
        if (not self.localization_client.wait_for_result(rospy.Duration(15.0))):
            self.localization_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Localization returned no result for object type " + object_type)
            return False

        try:
            res = self.localization_client.get_result()
            r = res.detected_poses[0]
            if r.poses.poses[0]:
                outpose = geometry_msgs.msg.PoseStamped()
                outpose.header = r.poses.header
                outpose.pose = r.poses.poses[0]
                return outpose
        except:
            pass
        rospy.logerr("Localization failed to find a pose for object type " + object_type)
        return False
