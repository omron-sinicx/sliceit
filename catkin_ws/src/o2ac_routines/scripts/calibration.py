#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, OMRON SINIC X
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
# Author: Felix von Drigalski, Cristian C. Beltran-Hernandez

import o2ac_routines.helpers as helpers
from o2ac_routines.common import O2ACCommon
import copy

import numpy as np
import rospy

import geometry_msgs.msg
import tf_conversions
from math import pi, radians

from ur_control import conversions
tau = 2.0*pi  # Part of math from Python 3.6


class CalibrationClass(O2ACCommon):
    """
    These routines check the robots' calibration by moving them to
    objects defined in the scene.
    """

    def __init__(self):
        super(CalibrationClass, self).__init__()

        self.a_bot_downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/2))
        self.bin_names = ["bin3_1", "bin2_4", "bin2_3", "bin2_2", "bin2_1", "bin1_2", "bin1_1", "bin1_4", "bin1_5", "bin1_3"]

        # Neutral downward in the taskboard frames
        rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    def offset_pose_in_own_coordinate_system(self, ps, offset):
        """
        ps is the PoseStamped to offset. offset is a Point.
        """
        rospy.loginfo("Received pose to offset to TCP link:")
        rospy.loginfo(str(ps.pose.position.x) + ", " + str(ps.pose.position.y) + ", " + str(ps.pose.position.z))
        rospy.loginfo(str(ps.pose.orientation.x) + ", " + str(ps.pose.orientation.y) + ", " + str(ps.pose.orientation.z) + ", " + str(ps.pose.orientation.w))

        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = ps.header.frame_id
        m.child_frame_id = "temp_pose__"
        m.transform.translation.x = ps.pose.position.x
        m.transform.translation.y = ps.pose.position.y
        m.transform.translation.z = ps.pose.position.z
        m.transform.rotation.x = ps.pose.orientation.x
        m.transform.rotation.y = ps.pose.orientation.y
        m.transform.rotation.z = ps.pose.orientation.z
        m.transform.rotation.w = ps.pose.orientation.w
        self.listener.setTransform(m)

        ps_with_offset = geometry_msgs.msg.PoseStamped()
        ps_with_offset.header.frame_id = "temp_pose__"
        ps_with_offset.pose.position.x = offset.x
        ps_with_offset.pose.position.y = offset.y
        ps_with_offset.pose.position.z = offset.z
        ps_with_offset.pose.orientation.w = 1.0

        ps_new = self.listener.transformPose(ps.header.frame_id, ps_with_offset)

        rospy.loginfo("New pose:")
        rospy.loginfo(str(ps_new.pose.position.x) + ", " + str(ps_new.pose.position.y) + ", " + str(ps_new.pose.position.z))
        rospy.loginfo(str(ps_new.pose.orientation.x) + ", " + str(ps_new.pose.orientation.y) + ", " + str(ps_new.pose.orientation.z) + ", " + str(ps_new.pose.orientation.w))

        return ps_new

    # TODO: Implement the above in the function below
    def cycle_through_calibration_poses(self, poses, robot_name, speed=0.1, with_approach=False, use_z_for_approach=False, move_lin=False, go_home=True, end_effector_link=""):
        home_pose = "home"
        if "screw" in end_effector_link:
            home_pose = "screw_ready"

        if with_approach and not use_z_for_approach:
            rospy.logwarn("with_approach only moves in the X direction of the header frame. Be careful.")

        for pose in poses:
            ps_approach = copy.deepcopy(pose)
            if not use_z_for_approach:
                ps_approach.pose.position.x -= .05
            else:
                ps_approach.pose.position.z += .05
            rospy.loginfo("============ Press `Enter` to move " + robot_name + " to " + pose.header.frame_id)
            helpers.publish_marker(pose, namespace="place_pose")
            input()
            robot = self.active_robots[robot_name]
            if go_home:
                robot.go_to_named_pose(home_pose)
            if with_approach:
                robot.go_to_pose_goal(ps_approach, speed=speed, end_effector_link=end_effector_link, move_lin=move_lin)
            if rospy.is_shutdown():
                break
            if with_approach:
                robot.go_to_pose_goal(ps_approach, speed=speed, end_effector_link=end_effector_link, move_lin=move_lin)
                robot.go_to_pose_goal(pose, speed=speed, end_effector_link=end_effector_link, move_lin=move_lin)
            else:
                robot.go_to_pose_goal(pose, speed=speed, end_effector_link=end_effector_link, move_lin=move_lin)

            rospy.loginfo("============ Press `Enter` to proceed ")
            input()
            if with_approach:
                robot.go_to_pose_goal(ps_approach, speed=speed, end_effector_link=end_effector_link, move_lin=move_lin)
            if go_home:
                robot.go_to_named_pose(home_pose, force_ur_script=move_lin)

        if go_home:
            rospy.loginfo("Moving all robots home again.")
            self.a_bot.go_to_named_pose("home")
            self.b_bot.go_to_named_pose("home")
            self.c_bot.go_to_named_pose("home")
        return

    def assembly_calibration_base_plate(self, robot_name="b_bot", end_effector_link="", panel_name="", rotation=0):
        # if not self.set_assembly("wrs_assembly_2020"):
        rospy.loginfo("============ Calibrating base plate for the assembly task. ============")
        rospy.loginfo("eef link " + end_effector_link + " should be 5 mm above each corner of the plate.")
        self.publish_part_in_assembled_position("base", marker_only=True)
        robot = self.active_robots[robot_name]

        self.make_space_for_robot(robot_name)

        if end_effector_link == "":
            robot.go_to_named_pose("home")
        elif "screw" in end_effector_link or "suction" in end_effector_link:
            robot.go_to_named_pose("screw_ready")

        poses = []
        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation.w = 1.0
        pose0.pose.position.x = -.007
        zero_rot = -tau/2 if robot_name == "a_bot" else 0
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(zero_rot + rotation, 0, 0))
        
        self.allow_collisions_with_robot_hand("base_fixture_top", robot_name)

        if panel_name == "motor_panel":
            for i in range(2):
                poses.append(copy.deepcopy(pose0))
            poses[0].header.frame_id = "assembled_part_01_screw_hole_panel2_1"
            poses[1].header.frame_id = "assembled_part_01_screw_hole_panel2_2"
        elif panel_name == "bearing_panel":
            for i in range(2):
                poses.append(copy.deepcopy(pose0))
            poses[0].header.frame_id = "assembled_part_01_screw_hole_panel1_1"
            poses[1].header.frame_id = "assembled_part_01_screw_hole_panel1_2"
        else:
            for i in range(4):
                poses.append(copy.deepcopy(pose0))
            poses[0].header.frame_id = "assembled_part_01_screw_hole_panel2_2"
            poses[1].header.frame_id = "assembled_part_01_screw_hole_panel2_1"
            poses[2].header.frame_id = "assembled_part_01_screw_hole_panel1_2"
            poses[3].header.frame_id = "assembled_part_01_screw_hole_panel1_1"

        print("moving to #", len(poses), "poses")

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, end_effector_link=end_effector_link, move_lin=True, with_approach=True)
        return

    def assembly_calibration_assembled_parts(self):
        rospy.loginfo("============ Calibrating full assembled parts for the assembly task. ============")
        rospy.loginfo("b_bot gripper tip should go close to some important spots.")
        poses = []

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation.w = 1.0
        pose0.pose.position.x = -.02

        for i in range(5):
            poses.append(copy.deepcopy(pose0))

        poses[1].header.frame_id = "assembled_part_03"   # Top of plate 2
        poses[1].pose.position.x = .058
        poses[1].pose.position.y = -.0025
        poses[1].pose.position.z = .095 + .01
        poses[1].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))

        poses[2].header.frame_id = "assembled_part_08_front_tip"  # Front of rotary shaft
        poses[2].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, -tau/2))
        poses[2].pose.position.x = .03

        poses[3].header.frame_id = "assembled_part_14_screw_head"
        poses[3].pose.position.x = -.03

        poses[4].header.frame_id = "assembled_part_04_tip"
        poses[4].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/2, 0, -tau/2))
        poses[4].pose.position.x = .03

        self.cycle_through_calibration_poses(poses, "b_bot", go_home=True)
        return

    def taskboard_calibration_with_tools(self, robot_name="b_bot", end_effector_link="", hole=""):
        rospy.loginfo("============ Calibrating taskboard screw holes. ============")
        rospy.loginfo("eef link " + end_effector_link + " should be 5 mm above each corner of the plate.")
        self.make_space_for_robot(robot_name)

        self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")
        # self.go_to_named_pose("home", robot_name)

        poses = []
        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation.w = 1.0
        pose0.pose.position.x = -.002
        if robot_name == "a_bot":
            pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
            if end_effector_link == "a_bot_gripper_tip_link":
                pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/4, 0, 0))
        elif robot_name == "b_bot":
            pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12, 0, 0))
            if end_effector_link == "b_bot_gripper_tip_link":
                pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/6, 0, 0))

        if not hole:
            for i in range(3):
                poses.append(copy.deepcopy(pose0))
            poses[0].header.frame_id = "taskboard_set_screw_link"
            poses[1].header.frame_id = "taskboard_m3_screw_link"
            poses[2].header.frame_id = "taskboard_m4_screw_link"
        elif hole == "m4":
            poses.append(copy.deepcopy(pose0))
            poses[0].header.frame_id = "taskboard_m4_screw_link"
            poses[0].pose.position.y += -0.001  # MAGIC NUMBER (points right)
            poses[0].pose.position.z += 0.002  # MAGIC NUMBER (points down)
        elif hole == "m3":
            poses.append(copy.deepcopy(pose0))
            poses[0].header.frame_id = "taskboard_m3_screw_link"
            poses[0].pose.position.y += 0.001  # MAGIC NUMBER (points right)
            poses[0].pose.position.z += -0.001  # MAGIC NUMBER (points down)
        elif hole == "setscrew":
            poses.append(copy.deepcopy(pose0))
            poses[0].header.frame_id = "taskboard_set_screw_link"
            poses[0].pose.position.y += -0.002  # MAGIC NUMBER (points right)
            poses[0].pose.position.z += 0.003  # MAGIC NUMBER (points down)

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, with_approach=True, end_effector_link=end_effector_link, move_lin=True)
        self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")
        return

    def tray_sponge_calibration(self, robot_name="a_bot", end_effector_link="a_bot_gripper_tip_link"):
        rospy.loginfo("============ Touching tray sponge. ============")
        rospy.loginfo("eef link " + end_effector_link + " should be touching the tray sponge in middle, then left, then right.")
        if robot_name == "a_bot":
            self.b_bot.go_to_named_pose("home")
        elif robot_name == "b_bot":
            self.a_bot.go_to_named_pose("home")

        poses = []
        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        pose0.header.frame_id = "tray_center"
        pose0.pose.position.z = 0.003

        for i in range(3):
            poses.append(copy.deepcopy(pose0))
        poses[0].pose.position.y = 0.0
        poses[1].pose.position.y = -0.1
        poses[2].pose.position.y = 0.1

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, with_approach=True, end_effector_link=end_effector_link, move_lin=True)
        return

    def tray_corners_calibration(self, robot_name="a_bot", end_effector_link="a_bot_gripper_tip_link"):
        rospy.loginfo("============ Touching tray corners. ============")
        rospy.loginfo("eef link " + end_effector_link + " should go to the tray corners.")
        if robot_name == "a_bot":
            self.b_bot.go_to_named_pose("home")
        elif robot_name == "b_bot":
            self.a_bot.go_to_named_pose("home")

        poses = []
        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        pose0.pose.position.z = 0.01

        for i in range(4):
            poses.append(copy.deepcopy(pose0))
        poses[0].header.frame_id = "traycorner_1_link"
        poses[1].header.frame_id = "traycorner_2_link"
        poses[2].header.frame_id = "traycorner_3_link"
        poses[3].header.frame_id = "traycorner_4_link"

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, with_approach=False, end_effector_link=end_effector_link, move_lin=True)
        return

    def touch_workspace_center(self):
        rospy.loginfo("============ Touching workspace center. ============")
        self.a_bot.go_to_named_pose("home")
        self.b_bot.go_to_named_pose("home")
        poses = []

        pose_a = geometry_msgs.msg.PoseStamped()
        pose_a.header.frame_id = "workspace_center"
        pose_a.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        pose_a.pose.position.x = .0
        pose_a.pose.position.y = -.2
        pose_a.pose.position.z = .03

        pose_b = copy.deepcopy(pose_a)
        pose_b.pose.position.x = .0
        pose_b.pose.position.y = .3
        pose_b.pose.position.z = .03
        pose_c = copy.deepcopy(pose_a)
        pose_c.pose.position.x = -.3
        pose_c.pose.position.y = .2
        pose_c.pose.position.z = .03

        rospy.loginfo("============ Going to 2 cm above the table. ============")
        self.b_bot.go_to_pose_goal(pose_b, speed=1.0)
        self.a_bot.go_to_pose_goal(pose_a, speed=1.0)

        rospy.loginfo("============ Press enter to go to .1 cm above the table. ============")
        i = input()
        if not rospy.is_shutdown():
            pose_a.pose.position.z = .001
            pose_b.pose.position.z = .001
            pose_c.pose.position.z = .001
            self.b_bot.go_to_pose_goal(pose_b, speed=0.01)
            self.a_bot.go_to_pose_goal(pose_a, speed=0.01)

        rospy.loginfo("============ Press enter to go home. ============")
        input()
        self.a_bot.go_to_named_pose("home")
        self.b_bot.go_to_named_pose("home")
        return

    def workspace_level_calibration(self, robot_name="a_bot", end_effector_link="a_bot_gripper_tip_link"):
        rospy.loginfo("============ Touching workspace surface (tray surface). ============")
        rospy.loginfo("eef link " + end_effector_link + " should be touching the surface where the tray is placed in middle, then left, then right.")
        if robot_name == "a_bot":
            self.b_bot.go_to_named_pose("home")
        elif robot_name == "b_bot":
            self.a_bot.go_to_named_pose("home")

        poses = []
        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        pose0.header.frame_id = "workspace_center"
        pose0.pose.position.x = 0.2
        pose0.pose.position.z = 0.005

        for i in range(2):
            poses.append(copy.deepcopy(pose0))
        # poses[0].pose.position.y = 0.0
        # poses[1].pose.position.y = -0.15
        # poses[2].pose.position.y = 0.15
        poses[0].pose.position.y = 0.15
        poses[1].pose.position.y = -0.15

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, with_approach=True, end_effector_link=end_effector_link, move_lin=True)
        return

    def make_space_for_robot(self, robot_name):
        if robot_name == "b_bot":
            self.a_bot.go_to_named_pose("home")
        elif robot_name == "a_bot":
            self.b_bot.go_to_named_pose("home")

    def screw_tool_test_assembly(self, robot_name="b_bot", tool_name="_screw_tool_m4_tip_link"):
        rospy.loginfo("============ Moving the screw tool m4 to the screw holes on the base plate ============")
        rospy.loginfo("============ The screw tool m4 has to be carried by the robot! ============")
        self.make_space_for_robot(robot_name)

        self.active_robots[robot_name].go_to_named_pose("screw_ready")
        poses = []

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.header.frame_id = "assembled_part_01_screw_hole_panel1_1"
        if robot_name == "b_bot":
            pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/8, 0, 0))
            pose0.pose.position.x -= .01

        for i in range(4):
            poses.append(copy.deepcopy(pose0))

        poses[1].header.frame_id = "assembled_part_01_screw_hole_panel1_2"
        poses[2].header.frame_id = "assembled_part_01_screw_hole_panel2_1"
        poses[3].header.frame_id = "assembled_part_01_screw_hole_panel2_2"
        end_effector_link = robot_name + tool_name
        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, move_lin=True, end_effector_link=end_effector_link)
        return

    def screw_action_test(self, robot_name="b_bot"):
        rospy.loginfo("============ Screwing in one of the plate screws with the tool using the action ============")
        rospy.loginfo("============ The screw tool m4 and a screw have to be carried by the robot! ============")
        self.active_robots[robot_name].go_to_named_pose("screw_ready")

        if robot_name == "b_bot":
            self.b_bot.go_to_named_pose("screw_plate_ready")

        pose0 = geometry_msgs.msg.PoseStamped()
        if robot_name == "b_bot":
            pose0.header.frame_id = "assembled_part_03_bottom_screw_hole_aligner_1"
            pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/8, 0, 0))
            pose0.pose.position.x = -.01

        self.skill_server.do_screw_action(robot_name, pose0, screw_size=4, screw_height=.02)
        self.active_robots[robot_name].go_to_named_pose("screw_plate_ready")
        return

    def screw_feeder_calibration(self, feeder_size="", robot_name="b_bot"):
        rospy.loginfo("============ Moving the screw tool m3 or m4 to its screw feeder ============")
        rospy.loginfo("============ The screw tool has to be carried by the robot! ============")

        self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready")

        pose0 = geometry_msgs.msg.PoseStamped()
        if robot_name == "a_bot":
            pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(radians(80), 0, 0))
            pose0.header.frame_id = "m3_feeder_outlet_link"
            ee_link = robot_name + "_screw_tool_m3_tip_link"
        else:
            pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
            pose0.header.frame_id = "m4_feeder_outlet_link"
            ee_link = robot_name + "_screw_tool_m4_tip_link"

        if feeder_size == "m3":
            pose0.header.frame_id = "m3_feeder_outlet_link"
            ee_link = robot_name + "_screw_tool_m3_tip_link"
        if feeder_size == "m4":
            pose0.header.frame_id = "m4_feeder_outlet_link"
            ee_link = robot_name + "_screw_tool_m4_tip_link"

        # MAGIC NUMBERS (needs to be synced with pick_screw_from_feeder_python)
        if robot_name == "a_bot" and feeder_size == "m4":
            rospy.logerr("APPLYING MAGIC NUMBERS")
            pose0.pose.position.y += -.004  # (world negative y-axis)
            pose0.pose.position.z += -.0035  # (world x-axis)

        poses = []
        for i in range(3):
            poses.append(copy.deepcopy(pose0))
        poses[0].pose.position.x = -.03
        poses[1].pose.position.x = 0
        poses[2].pose.position.x = -.01

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, move_lin=True, with_approach=False, end_effector_link=ee_link)
        # self.skill_server.toggle_collisions(collisions_on=True)
        self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready")
        return

    def screw_feeder_pick_test(self, robot_name="b_bot", screw_size=4):
        rospy.loginfo("============ Picking a screw from a feeder ============")
        rospy.loginfo("============ The screw tool has to be carried by the robot! ============")

        self.pick_screw_from_feeder(robot_name, screw_size=screw_size, realign_tool_upon_failure=False)
        return

    def vertical_plate_screw_position_test(self, panel, robot_name="b_bot"):
        self.set_assembly("wrs_assembly_2021")
        """panel should be motor_plate or bearing_plate"""
        rospy.loginfo("============ Move tool to screw position of plates ============")

        self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready")

        pose0 = geometry_msgs.msg.PoseStamped()
        if robot_name == "a_bot":
            pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(radians(20), 0, 0))
            ee_link = robot_name + "_screw_tool_m4_tip_link"
        else:
            pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-radians(20), 0, 0))
            pose0.header.frame_id = "bottom_screw_hole_2"
            ee_link = robot_name + "_screw_tool_m4_tip_link"
        if panel == "bearing_panel":
            part_name = "assembled_part_03_"
        elif panel == "motor_panel":
            part_name = "assembled_part_02_"

        pose0.header.frame_id = part_name + "bottom_screw_hole_1"

        # self.toggle_collisions(collisions_on=False)

        poses = []
        for i in range(2):
            poses.append(copy.deepcopy(pose0))
        poses[1].header.frame_id = part_name + "bottom_screw_hole_2"

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, move_lin=True, end_effector_link=ee_link)
        # self.toggle_collisions(collisions_on=True)
        self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready")
        return

    def bearing_screw_holes(self, robot_name="b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link"):
        rospy.loginfo("============ Move tool to bearing screw holes with tool ============")

        self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")

        poses = []
        for i in [1, 2, 3, 4]:
            screw_pose = geometry_msgs.msg.PoseStamped()
            screw_pose.header.frame_id = "assembled_part_07_screw_hole_" + str(i)
            rospy.logwarn("Is this magic number the same as in common.py? Confirm manually!")
            screw_pose.pose.position.z += .0025  # MAGIC NUMBER
            screw_pose.pose.position.x += .005
            if robot_name == "a_bot":
                screw_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
            else:
                screw_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12, 0, 0))

            screw_pose_approach = copy.deepcopy(screw_pose)
            screw_pose_approach.pose.position.x -= 0.07
            poses.append(screw_pose_approach)
            poses.append(screw_pose)
            poses.append(screw_pose_approach)

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, move_lin=True, end_effector_link=end_effector_link)
        self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")
        return

    def motor_screw_holes(self, robot_name="a_bot", end_effector_link="b_bot_screw_tool_m4_tip_link"):
        rospy.loginfo("============ Move tool to bearing screw holes with tool ============")

        self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")

        poses = []
        for i in [1, 2, 3, 4, 5, 6]:
            # TODO(felixvd): Do not use the orientation defined in these frames. Get a static orientation.
            screw_pose = geometry_msgs.msg.PoseStamped()
            screw_pose.header.frame_id = "assembled_part_02_motor_screw_hole_" + str(i)
            screw_pose.pose.position.z += -.001  # MAGIC NUMBER
            screw_pose.pose.position.x += .005
            if robot_name == "a_bot":
                screw_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12+tau/2, 0, 0))
            else:
                screw_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12+tau/2, 0, 0))

            screw_pose_approach = copy.deepcopy(screw_pose)
            screw_pose_approach.pose.position.x -= 0.07
            poses.append(screw_pose_approach)
            poses.append(screw_pose)
            poses.append(screw_pose_approach)

        self.cycle_through_calibration_poses(poses, robot_name, go_home=False, move_lin=True, end_effector_link=end_effector_link)
        self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")
        return

    def go_to_tool_pickup_pose(self, robot_name="b_bot", screw_tool_id="screw_tool_m4"):
        self.b_bot.go_to_named_pose("tool_pick_ready")
        ps_tool_pickup = geometry_msgs.msg.PoseStamped()
        ps_tool_pickup.header.frame_id = screw_tool_id + "_pickup_link"
        ps_tool_pickup.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi/6, 0))
        ps_tool_pickup.pose.position.x = .017
        ps_tool_pickup.pose.position.z = -.008
        self.b_bot.go_to_pose_goal(ps_tool_pickup, speed=.1, acceleration=.02)

    def calibrate_place_panel(self, panel_name):
        self.a_bot.gripper.open()
        self.a_bot.move_lin_rel([0, 0, 0.1])
        self.a_bot.go_to_named_pose("centering_area")
        self.place_panel("a_bot", panel_name, pick_again=True, fake_position=True)

    def motor_insertion_from_aid(self):
        self.b_bot.gripper.open()
        self.b_bot.move_lin_rel([0, 0, 0.2])
        self.b_bot.go_to_named_pose("centering_area")
        self.align_motor_pre_insertion(False)
        print("current pose", np.round(conversions.from_pose_to_list(self.listener.transformPose("assembled_part_02_back_hole", self.a_bot.get_current_pose_stamped()).pose), 4))
        self.confirm_to_proceed("finetune")
        self.insert_motor("assembled_part_02_back_hole")
        self.confirm_to_proceed("are we done?")

    def bearing_insertion_assembly(self):
        self.a_bot.gripper.open()
        self.a_bot.move_lin_rel([0, 0, 0.1])
        self.a_bot.go_to_named_pose("home")
        self.pick_bearing("a_bot")
        self.orient_bearing(task="assembly", robot_name="a_bot")
        print("current pose", np.round(conversions.from_pose_to_list(self.listener.transformPose("assembled_part_07_inserted", self.a_bot.get_current_pose_stamped()).pose), 4))
        self.confirm_to_proceed("finetune")
        self.insert_bearing("assembled_part_07_inserted", robot_name="a_bot")
        print("current pose", np.round(conversions.from_pose_to_list(self.listener.transformPose("assembled_part_07_inserted", self.a_bot.get_current_pose_stamped()).pose), 4))
        self.confirm_to_proceed("we are done?")

    def motor_pulley_insertion(self):
        self.a_bot.gripper.open()
        self.a_bot.move_lin_rel([0, 0, 0.1])
        self.a_bot.go_to_named_pose("home")
        self.is_motor_pulley_in_storage = False
        self.pick_motor_pulley(robot_name="a_bot")
        self.orient_motor_pulley("assembled_part_05_center", robot_name="a_bot")
        print("current pose", np.round(conversions.from_pose_to_list(self.listener.transformPose("assembled_part_05_center", self.a_bot.get_current_pose_stamped()).pose), 4))
        self.confirm_to_proceed("finetune")
        self.insert_motor_pulley("assembled_part_05_center", robot_name="a_bot", retry_insertion=True)
        print("current pose", conversions.from_pose_to_list(self.listener.transformPose("assembled_part_05_center", self.a_bot.get_current_pose_stamped()).pose))
        self.confirm_to_proceed("are we done?")

    def motor_pulley_fastening(self):
        self.a_bot.gripper.open()
        self.fasten_motor_pulley("assembled_part_05_center")

    def end_cap_and_shaft_prep(self):
        self.pick_end_cap()
        self.orient_shaft_end_cap(calibration=True)
        self.pick_shaft()
        self.orient_shaft(calibration=True)

    def end_cap_and_shaft_preinsertion(self):
        pre_insertion_shaft = [1.78158, -0.98719, 2.42349, -4.57638, -1.78597, 0.00433]
        if not self.b_bot.move_joints(pre_insertion_shaft, speed=0.4):
            rospy.logerr("Fail to go to pre_insertion_shaft")
            return False

        above_pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [-0.004, 0.010, 0.280]+np.deg2rad([-180, 90, -90]).tolist())
        if not self.a_bot.go_to_pose_goal(above_pre_insertion_end_cap, speed=0.6, move_lin=False):
            rospy.logerr("Fail to go to pre_insertion_end_cap")
            return False
        pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [-0.0045, 0.010, 0.245]+np.deg2rad([-180, 90, -90]).tolist())
        if not self.a_bot.go_to_pose_goal(pre_insertion_end_cap, speed=0.3, move_lin=True):
            rospy.logerr("Fail to go to pre_insertion_end_cap")
            return False

    def simple_end_cap_pick(self):
        self.a_bot.go_to_named_pose("centering_area")
        self.orient_shaft_end_cap(ignore_orientation=True)

    def end_cap_insertion(self):
        self.insert_end_cap()
        self.a_bot.gripper.send_command(0.06, velocity=0.01)
        self.a_bot.move_lin_rel([0, 0, 0.05], speed=0.3)

    def bearing_spacer(self):
        self.pick_bearing_spacer("a_bot")
        self.orient_bearing_spacer("a_bot")
        self.align_bearing_spacer_pre_insertion(robot_name="a_bot")
        self.confirm_to_proceed("finetune")
        self.insert_bearing_spacer("assembled_part_07_inserted", "a_bot")
        self.confirm_to_proceed("okay?")

    def output_pulley(self):
        self.pick_output_pulley("a_bot")
        self.orient_output_pulley("a_bot")
        self.align_output_pulley_pre_insertion("a_bot")
        self.confirm_to_proceed("finetune")
        self.insert_output_pulley("assembled_part_07_inserted", "a_bot")
        self.confirm_to_proceed("okay?")


if __name__ == '__main__':
    try:
        rospy.init_node('o2ac_routines', anonymous=False)
        c = CalibrationClass()
        # c.set_assembly("wrs_assembly_2021")

        while not rospy.is_shutdown():
            rospy.loginfo("============ Calibration procedures ============ ")
            rospy.loginfo("Enter a number to check calibrations for the following things: ")
            rospy.loginfo("1: home (b_bot), 100/101: home/back (both robots)")
            rospy.loginfo("11: b_bot_outside_camera, 12: b_bot_inside_camera (activate)")
            rospy.loginfo("13, 14: Equip/unequip m3 screw tool with a_bot")
            rospy.loginfo("15, 16: Equip/unequip m4 screw tool with b_bot")
            rospy.loginfo("17, 18: Equip/unequip set screw tool with b_bot")
            rospy.loginfo("===== GENERAL")
            rospy.loginfo("291, 292: Touch tray sponge with a_bot, b_bot")
            rospy.loginfo("293, 294: Touch tray corners with a_bot, b_bot")
            rospy.loginfo("21, 22: Calibrate screw feeders (a_bot, b_bot)")
            rospy.loginfo("23, 24: Pick m3/m4 screw from feeder")
            rospy.loginfo("===== TASKBOARD TASK")
            rospy.loginfo("31, 32: Go to screw holes with a_bot m3, b_bot m4")
            rospy.loginfo("===== ASSEMBLY TASK (no parts may be mounted!)")
            rospy.loginfo("501-502: Base plate screw holes (a_bot, b_bot)")
            rospy.loginfo("503-504: Base plate screw holes (a_bot m3, b_bot m4)")
            rospy.loginfo("511-512: Motor plate holes (b_bot, b_bot m4)")
            rospy.loginfo("55: Bearing screw holes (b_bot m4)")
            rospy.loginfo("60: L-plates")
            rospy.loginfo("===== TOOLS ")
            # rospy.loginfo("65 (651/652): Go to belt tool pickup position (and equip/unequip it)")
            # rospy.loginfo("66 (661/662): Go to plunger tool pickup position (and equip/unequip it)")
            rospy.loginfo("70: Do screw action with b_bot on rightmost hole")
            rospy.loginfo("81: Realign M4 tool")
            rospy.loginfo("x: Exit ")
            rospy.loginfo("reset: reset the scene")
            rospy.loginfo(" ")
            r = input()
            if r == 'b':
                c.publish_part_in_assembled_position("base", marker_only=True)
            if r == '1':
                c.a_bot.go_to_named_pose("home")
                c.b_bot.go_to_named_pose("home")
            if r == '1000':  # Speed test
                ps = geometry_msgs.msg.PoseStamped()
                ps.header.frame_id = "workspace_center"
                ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/2))
                ps.pose.position.z = .3
                # c.b_bot.go_to_named_pose("home")
                # c.b_bot.move_lin(ps, speed=.2)
                # c.b_bot.go_to_named_pose("home", speed=0.3)
                # c.b_bot.move_lin(ps, speed=.5)
                # c.b_bot.go_to_named_pose("home", speed=0.6)
                c.b_bot.move_lin(ps, speed=.5, acceleration=0.8)
                c.b_bot.go_to_named_pose("home", speed=0.5, acceleration=0.8)
            if r == '100':
                c.a_bot.go_to_named_pose("home")
                c.b_bot.go_to_named_pose("home")
            if r == '101':
                c.a_bot.go_to_named_pose("back")
                c.b_bot.go_to_named_pose("back")
            if r == '102':
                c.b_bot.go_to_named_pose("back", speed=1.0, acceleration=0.7)
                c.b_bot.go_to_named_pose("home", speed=1.0, acceleration=0.7)
            if r == '11':
                c.vision.activate_camera("b_bot_outside_camera")
            if r == '12':
                c.vision.activate_camera("b_bot_inside_camera")
            if r == '11a':
                c.vision.activate_camera("a_bot_outside_camera")
            if r == '12a':
                c.vision.activate_camera("a_bot_inside_camera")
            if r == "13":
                c.make_space_for_robot("a_bot")
                c.equip_tool("a_bot", "screw_tool_m3")
            if r == "13b":
                c.make_space_for_robot("b_bot")
                c.equip_tool("b_bot", "screw_tool_m3")
            if r == "14b":
                c.make_space_for_robot("b_bot")
                c.unequip_tool("b_bot", "screw_tool_m3")
            if r == "14":
                c.make_space_for_robot("a_bot")
                c.unequip_tool("a_bot", "screw_tool_m3")
            if r == "15":
                c.make_space_for_robot("b_bot")
                c.equip_tool("b_bot", "screw_tool_m4")
            if r == "16":
                c.make_space_for_robot("b_bot")
                c.unequip_tool("b_bot", "screw_tool_m4")
            if r == "15a":
                c.make_space_for_robot("a_bot")
                c.equip_tool("a_bot", "screw_tool_m4")
            if r == "16a":
                c.make_space_for_robot("a_bot")
                c.unequip_tool("a_bot", "screw_tool_m4")
            if r == "17":
                c.make_space_for_robot("b_bot")
                c.equip_tool("b_bot", "set_screw_tool")
            if r == "18":
                c.make_space_for_robot("b_bot")
                c.unequip_tool("b_bot", "set_screw_tool")
            if r == "191":
                c.equip_tool("b_bot", "padless_tool_m4")
            if r == "192":
                c.unequip_tool("b_bot", "padless_tool_m4")
            if r == '21':
                c.screw_feeder_calibration(robot_name="a_bot")
            if r == 'am3':
                c.equip_unequip_realign_tool("a_bot", "screw_tool_m3", "equip", calibration_mode=True)
            if r == 'am4':
                c.equip_unequip_realign_tool("a_bot", "screw_tool_m4", "equip", calibration_mode=True)
            if r == 'bm4':
                c.equip_unequip_realign_tool("b_bot", "screw_tool_m4", "equip", calibration_mode=True)
            if r == 'bm3':
                c.equip_unequip_realign_tool("b_bot", "screw_tool_m3", "equip", calibration_mode=True)
            if r == '210':
                c.calibration_mode = True
                c.pick_screw_from_feeder_python(robot_name="a_bot", screw_size=3, realign_tool_upon_failure=False, skip_retreat=False)
                c.calibration_mode = False
            if r == '22':
                c.screw_feeder_calibration(robot_name="b_bot")
            if r == '220':
                c.calibration_mode = True
                c.pick_screw_from_feeder_python(robot_name="b_bot", screw_size=4, realign_tool_upon_failure=False, skip_retreat=False)
                c.calibration_mode = False
            if r == '22a':
                c.screw_feeder_calibration(robot_name="a_bot", feeder_size="m4")
            if r == '220a':
                c.calibration_mode = True
                c.pick_screw_from_feeder_python(robot_name="a_bot", screw_size=4, realign_tool_upon_failure=False, skip_retreat=False)
                c.calibration_mode = False
            if r == '23':
                c.screw_feeder_pick_test(robot_name="a_bot", screw_size=3)
            if r == '24':
                c.screw_feeder_pick_test(robot_name="b_bot", screw_size=4)
            if r == '24a':
                c.screw_feeder_pick_test(robot_name="a_bot", screw_size=4)
            if r == '281':
                c.planning_scene_interface.allow_collisions("tray")
                c.planning_scene_interface.allow_collisions("tray_center")
                c.workspace_level_calibration(robot_name="a_bot", end_effector_link="a_bot_gripper_tip_link")
            if r == '282':
                c.planning_scene_interface.allow_collisions("tray")
                c.planning_scene_interface.allow_collisions("tray_center")
                c.workspace_level_calibration(robot_name="b_bot", end_effector_link="b_bot_gripper_tip_link")
            if r == '291':
                c.tray_sponge_calibration(robot_name="a_bot", end_effector_link="a_bot_gripper_tip_link")
            if r == '292':
                c.tray_sponge_calibration(robot_name="b_bot", end_effector_link="b_bot_gripper_tip_link")
            if r == '293':
                c.tray_corners_calibration(robot_name="a_bot", end_effector_link="a_bot_gripper_tip_link")
            if r == '294':
                c.tray_corners_calibration(robot_name="b_bot", end_effector_link="b_bot_gripper_tip_link")
            if r == '31':
                c.taskboard_calibration_with_tools(robot_name="a_bot", end_effector_link="a_bot_screw_tool_m3_tip_link", hole="m3")
            if r == '311':
                c.taskboard_calibration_with_tools(robot_name="a_bot", end_effector_link="a_bot_gripper_tip_link")
            if r == '32':
                c.taskboard_calibration_with_tools(robot_name="b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link", hole="m4")
            if r == '321':
                c.taskboard_calibration_with_tools(robot_name="b_bot", end_effector_link="b_bot_gripper_tip_link")
            if r == '33':
                c.taskboard_calibration_with_tools(robot_name="b_bot", end_effector_link="b_bot_set_screw_tool_tip_link", hole="setscrew")
            if r == '501':
                c.assembly_calibration_base_plate("a_bot", panel_name="motor_panel")
            if r == '5011':
                c.assembly_calibration_base_plate("a_bot", panel_name="bearing_panel")
            if r == '5012':
                c.assembly_calibration_base_plate("a_bot", panel_name="motor_panel", rotation=tau/4)
            if r == '502':
                c.assembly_calibration_base_plate("b_bot", panel_name="motor_panel")
            if r == '5021':
                c.assembly_calibration_base_plate("b_bot", panel_name="bearing_panel")
            if r == '5022':
                c.assembly_calibration_base_plate("b_bot", panel_name="motor_panel", rotation=tau/4)
            if r == '5023':
                c.assembly_calibration_base_plate("b_bot", panel_name="both")
            if r == '503':
                c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m3_tip_link", panel_name="motor_panel")
            if r == '5031':
                c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m3_tip_link", panel_name="bearing_panel")
            if r == '5032':
                c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m3_tip_link", panel_name="motor_panel", rotation=radians(25))
            if r == '5033':
                c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m3_tip_link", panel_name="motor_panel", rotation=radians(45))
            if r == '504':
                c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link", panel_name="both")
            if r == '5041':
                c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link", panel_name="both", rotation=radians(25))
            if r == '5042':
                c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link", panel_name="both", rotation=radians(45))
            if r == '5043':
                c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link", panel_name="both", rotation=radians(-45))
            if r == '504a':
                c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m4_tip_link", panel_name="both")
            if r == '5042a':
                c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m4_tip_link", panel_name="both", rotation=radians(25))
            if r == '5043a':
                c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m4_tip_link", panel_name="both", rotation=radians(-45))
            if r == '504a':
                c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m4_tip_link")
            if r == '503b':
                c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m3_tip_link")

            if r == '511':
                c.assembly_calibration_base_plate("b_bot", context="motor_plate")
            if r == '52':
                c.vertical_plate_screw_position_test(panel="motor_panel")
            if r == '522':
                c.a_bot.move_lin_rel([-0.05, 0, 0])
                c.a_bot.move_lin_rel([0.05, 0, 0])
            if r == '53':  # Bearing rotation
                ps = geometry_msgs.msg.PoseStamped()

                ps.header.frame_id = "assembled_part_07_inserted"
                ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, radians(0), 0))
                ps.pose.position = geometry_msgs.msg.Point(-0.0, 0.0, 0.0)
                c.b_bot.go_to_pose_goal(ps, speed=.1, acceleration=.04, end_effector_link="b_bot_bearing_rotate_helper_link")
                c.b_bot.gripper.close()
                ps.pose.orientation = helpers.rotateQuaternionByRPYInUnrotatedFrame(radians(45), 0, 0, ps.pose.orientation)
                c.b_bot.go_to_pose_goal(ps, speed=.1, acceleration=.04, end_effector_link="b_bot_bearing_rotate_helper_link")
                c.b_bot.gripper.open()
                ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
                c.b_bot.go_to_pose_goal(ps, speed=.1, acceleration=.04, end_effector_link="b_bot_bearing_rotate_helper_link")
                ps.pose.orientation = helpers.rotateQuaternionByRPYInUnrotatedFrame(radians(-45), 0, 0, ps.pose.orientation)
                c.b_bot.gripper.close()
                c.b_bot.go_to_pose_goal(ps, speed=.1, acceleration=.04, end_effector_link="b_bot_bearing_rotate_helper_link")
                ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
                c.b_bot.gripper.open()
                c.b_bot.go_to_pose_goal(ps, speed=.1, acceleration=.04, end_effector_link="b_bot_bearing_rotate_helper_link")
            if r == '531':  # Bearing rotation
                c.align_bearing_holes(max_adjustments=10, task="assembly")
            if r == '54':  # Motor angle
                c.vision.activate_camera("b_bot_outside_camera")
                camera_look_pose = geometry_msgs.msg.PoseStamped()
                camera_look_pose.header.frame_id = "vgroove_aid_link"
                camera_look_pose.pose.orientation = geometry_msgs.msg.Quaternion(*(0.84, 0.0043246, 0.0024908, 0.54257))
                camera_look_pose.pose.position = geometry_msgs.msg.Point(-0.0118, 0.133, 0.0851)
                camera_look_pose.pose.position.z += 0.2
                c.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_outside_camera_color_optical_frame", speed=.1, acceleration=.04)
                camera_look_pose.pose.position.z -= 0.2
                c.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_outside_camera_color_optical_frame", speed=.1, acceleration=.04)
            if r == '55':  # Bearing screw holes
                c.bearing_screw_holes()
            if r == '551':  # Bearing screw holes
                c.motor_screw_holes(robot_name="b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link")
            if r == '544':
                c.vision.activate_camera("b_bot_outside_camera")
                angle = c.get_motor_angle()
            if r == '600':
                c.a_bot.gripper.open()
                c.a_bot.move_lin_rel([0, 0, 0.1])
            if r == '600b':
                c.b_bot.gripper.open()
                c.b_bot.move_lin_rel([0, 0, 0.1])
            if r == '601':
                c.calibrate_place_panel("panel_bearing")
            if r == '601f':
                c.fasten_panel('panel_bearing')
            if r == '601fc':
                c.calibration_mode = True
                c.fasten_panel('panel_bearing')
                c.calibration_mode = False
            if r == '601b':  # Fallback to reposition
                c.center_panel_on_base_plate("panel_bearing", calibration=True)
            if r == '602':
                c.calibrate_place_panel("panel_motor")
            if r == '602f':
                c.fasten_panel('panel_motor')
            if r == '602fc':
                c.calibration_mode = True
                c.fasten_panel('panel_motor')
                c.calibration_mode = False
            if r == '602b':  # Fallback to reposition
                c.center_panel_on_base_plate("panel_motor", calibration=True)
            if r == '603p':
                c.pick_and_orient_motor()
            if r == '603o':
                c.orient_motor()
            if r == '603':
                c.motor_insertion_from_aid()
            if r == '603f':
                c.fasten_motor()
            if r == '603f2':
                c.fasten_motor(part1=False, part2=True)
            if r == '603f1c':
                c.calibration_mode = True
                c.fasten_motor(part1=True, part2=False)
                c.calibration_mode = False
            if r == '603f2c':
                c.calibration_mode = True
                c.fasten_motor(part1=False, part2=True)
                c.calibration_mode = False
            if r == '604':
                c.bearing_insertion_assembly()
            if r == '604i':
                c.orient_bearing(task="assembly", robot_name="a_bot", part1=False)
                print("current pose", conversions.from_pose_to_list(c.listener.transformPose("assembled_part_07_inserted", c.a_bot.get_current_pose_stamped()).pose))
                c.confirm_to_proceed("finetune")
                c.insert_bearing("assembled_part_07_inserted", robot_name="a_bot")
                print("current pose", conversions.from_pose_to_list(c.listener.transformPose("assembled_part_07_inserted", c.a_bot.get_current_pose_stamped()).pose))
                c.confirm_to_proceed("we are done?")
            if r == '604f':
                c.fasten_bearing(task="assembly", robot_name="a_bot")
            if r == '604fb':
                c.fasten_bearing(task="assembly", robot_name="b_bot")
            if r == '605':
                c.motor_pulley_insertion()
            if r == '606':
                c.motor_pulley_fastening()
            if r == '607':
                c.end_cap_and_shaft_prep()
            if r == '607sp':
                c.pick_shaft()
            if r == '607so':
                c.orient_shaft(calibration=True, ignore_orientation=False)
            if r == '607so2':
                c.orient_shaft(calibration=True, ignore_orientation=True)
            if r == '607so3':
                pre_insertion_shaft = [1.78158, -0.98719, 2.42349, -4.57638, -1.78597, 0.00433]
                c.b_bot.move_joints(pre_insertion_shaft, speed=0.4)
            if r == '607so4':
                c.check_screw_hole_visible_on_shaft_in_v_groove()
            if r == '608':
                c.end_cap_and_shaft_preinsertion()
            if r == '609':
                c.simple_end_cap_pick()
            if r == '610':
                c.end_cap_insertion()
            if r == '611':
                c.fasten_end_cap()
            if r == '612':
                c.align_shaft("assembled_part_07_inserted", pre_insert_offset=0.065)
                c.insert_shaft("assembled_part_07_inserted", target=0.043)
            if r == '613':
                c.bearing_spacer()
            if r == '614':
                c.output_pulley()
            if r == '614f':
                c.fasten_output_pulley()
            if r == '614c':
                c.check_output_pulley_angle()
            if r == '615r':
                c.a_bot.gripper.open(opening_width=0.03)
                c.a_bot.move_lin_rel([-0.15,0,0], relative_to_tcp=True, speed=0.2)
                c.b_bot.gripper.open(opening_width=0.03)
                c.b_bot.move_lin_rel([-0.15,0,0], relative_to_tcp=True, speed=0.2)
            if r == '615':
                c.publish_part_in_assembled_position("base")
                c.publish_part_in_assembled_position("panel_motor")
                c.publish_part_in_assembled_position("motor")
                c.insert_motor_cables_without_tools_normal("black")
            if r == '615t':
                c.publish_part_in_assembled_position("base")
                c.publish_part_in_assembled_position("panel_motor")
                c.publish_part_in_assembled_position("motor")
                c.insert_motor_cables_with_tool("black")
            if r == '616':
                c.publish_part_in_assembled_position("base")
                c.publish_part_in_assembled_position("panel_motor")
                c.publish_part_in_assembled_position("motor")
                c.insert_motor_cables_without_tools_normal("red")
            if r == '616t':
                c.publish_part_in_assembled_position("base")
                c.publish_part_in_assembled_position("panel_motor")
                c.publish_part_in_assembled_position("motor")
                c.insert_motor_cables_with_tool("red")
            if r == '617':
                c.calibration_mode = True
                c.equip_cable_tool()
                c.calibration_mode = False
            if r == '618':
                c.calibration_mode = True
                c.unequip_cable_tool()
                c.calibration_mode = False
            if r == '6':
                c.a_bot.go_to_named_pose("back")
                c.b_bot.go_to_named_pose("screw_ready")
            if r == '65':
                c.b_bot.go_to_named_pose("home")
                ps = geometry_msgs.msg.PoseStamped()
                ps.header.frame_id = "belt_tool_pickup_link"
                ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
                ps.pose.position.x = -.05
                c.b_bot.go_to_pose_goal(ps, speed=.2, acceleration=.04)
            if r == '651':
                c.do_change_tool_action("b_bot", equip=True, screw_size=100)
            if r == '66':
                c.b_bot.go_to_named_pose("home")
                ps = geometry_msgs.msg.PoseStamped()
                ps.header.frame_id = "plunger_tool_pickup_link"
                ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
                ps.pose.position.x = -.05
                c.b_bot.go_to_pose_goal(ps, speed=.2, acceleration=.04)
            if r == '661':
                c.do_change_tool_action("b_bot", equip=True, screw_size=200)
            if r == '63':
                c.screw_tool_test_assembly(robot_name="b_bot")
            if r == '70':
                c.screw_action_test(robot_name="b_bot")
            if r == '81':
                c.realign_tool(robot_name="b_bot", screw_tool_id="screw_tool_m4")
            if r == "reset":
                c.reset_scene_and_robots()
                c.reset_assembly_visualization()
            if r == "activate":
                c.a_bot.activate_ros_control_on_ur()
                c.b_bot.activate_ros_control_on_ur()
            if r == "load2020":
                c.assembly_status.tray_placed_on_table = True
                c.set_assembly("wrs_assembly_2020")
            if r == "load2021":
                c.assembly_status.tray_placed_on_table = True
                c.set_assembly("wrs_assembly_2021")
            if r == "load2021s":
                c.assembly_status.tray_placed_on_table = True
                c.set_assembly("wrs_assembly_2021_surprise")
            if r == 'x':
                break
        rospy.loginfo("============ Exiting!")

    except rospy.ROSInterruptException:
        print ("Something went wrong.")
