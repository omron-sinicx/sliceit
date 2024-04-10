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
# Author: Felix von Drigalski

import signal
import o2ac_routines.helpers as helpers
from o2ac_routines.assembly import O2ACAssembly
import time
import sys
import rospy
import copy
import numpy as np
from ur_control import conversions, transformations
from math import degrees, pi, radians, cos, sin
tau = 2.0*pi  # Part of math from Python 3.6

# For writing the camera image
import cv_bridge  # This offers conversion methods between OpenCV
import cv2
from datetime import datetime
import os
import sensor_msgs

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class IHPEExperiment2021(O2ACAssembly):

    def __init__(self):
        super(O2ACAssembly, self).__init__()
        self.set_assembly("wrs_assembly_2021")

        # For recording the camera image
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/extra_camera/color/image_raw", sensor_msgs.msg.Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/b_bot_outside_camera/color/image_raw", sensor_msgs.msg.Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/a_bot_inside_camera/color/image_raw", sensor_msgs.msg.Image, self.image_callback)
        self.cam_image = []
    
    def image_callback(self, msg):
        self.cam_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def write_cam_image(self, image_name="b_bot_cam"):
        now = datetime.now()
        timesuffix = now.strftime("%Y-%m-%d_%H:%M:%S")
        folder = os.path.join(self.rospack.get_path("o2ac_routines"), "log")
        # print ("IMG:", self.cam_image)
        try:
            cv2.imwrite(os.path.join(folder, image_name + "_" + timesuffix + ".png"), np.array(self.cam_image))
        except:
            rospy.logerr("Failed to save camera image")

    def spawn_panel(self, panel_name="panel_bearing", pose=[0, 0.03, 0.0, tau/4, 0, -tau/4]):
        co = copy.deepcopy(self.assembly_database.get_collision_object(panel_name))
        if co:
            co.header.frame_id = "tray_center"
            co.pose = conversions.to_pose(pose)  # Convert to geometry_msgs.msg.Pose
            self.planning_scene_interface.apply_collision_object(co)
        else:
            rospy.logerr("Could not find object with name " + panel_name + ". Did not spawn to planning scene.")

        self.planning_scene_interface.allow_collisions(panel_name)  # Workaround

    def return_panel_to_tray(self, panel_name="panel_bearing", add_random_offset=False):
        # The poses on the base plate
        place_pose = self.get_panel_place_pose(panel_name)
        above_place_pose = copy.deepcopy(place_pose)
        above_place_pose.pose.position.x = -0.100

        self.a_bot.gripper.close()
        self.a_bot.move_lin(above_place_pose, speed=0.4)
        
        # The poses in the tray
        self.despawn_object(panel_name)
        c.spawn_panel(panel_name)
        grasp_pose = self.get_panel_grasp_pose(panel_name)
        grasp_pose = helpers.rotatePoseByRPY(-135, 0, 0, grasp_pose)
        grasp_pose.header.frame_id = "move_group/" + panel_name
        grasp_pose_tray = self.transform_pose_with_wait(grasp_pose)
        if panel_name == "panel_bearing":
            grasp_depth_offset = 0.04
        elif panel_name == "panel_motor":
            grasp_depth_offset = 0.03
        grasp_pose_tray.pose.position.z -= grasp_depth_offset

        if add_random_offset:
            if panel_name == "panel_bearing":
                angle_range = 15
                xy_range = 0.03
            elif panel_name == "panel_motor":  
                angle_range = 10
                xy_range = 0.015
            offset_angle = np.random.uniform(radians(-angle_range), radians(angle_range))
            offset_x = np.random.uniform(-xy_range, xy_range)
            offset_y = np.random.uniform(-xy_range, xy_range)
            grasp_pose_tray = helpers.rotatePoseByRPY(offset_angle, 0, 0, grasp_pose_tray)
            grasp_pose_tray.pose.position.x += offset_x
            grasp_pose_tray.pose.position.y += offset_y
        
        above_grasp_pose_tray = copy.deepcopy(grasp_pose_tray)
        above_grasp_pose_tray.pose.position.z += 0.1
        self.a_bot.move_lin(above_grasp_pose_tray, speed=0.5)
        self.a_bot.move_lin(grasp_pose_tray, speed=0.2)
        self.a_bot.gripper.open(opening_width=0.06)
        self.a_bot.move_lin(above_grasp_pose_tray, speed=0.5)
        if add_random_offset:
            return offset_angle, offset_x, offset_y
        return

    def transform_pose_with_wait(self, pose, target_frame="tray_center"):
        try:
            self.listener.waitForTransform(pose.header.frame_id, target_frame, pose.header.stamp, rospy.Duration(2))
            return self.listener.transformPose(target_frame, pose)
        except:
            rospy.logerr("Could not transform to frame: " + pose.header.frame_id)
            return False
        
    def get_panel_place_pose(self, panel_name):
        """ Define the Magic numbers for placing the L-plates on the base plate"""
        if panel_name == "panel_bearing":
            object_frame = "assembled_part_01_screw_hole_panel1_1"
            l_plate = 0.116
            offset_height = -0.017
        else:  # panel_motor
            object_frame = "assembled_part_01_screw_hole_panel2_1"
            l_plate = 0.06
            offset_height = -0.0088  # Adjusted, because we grasp the plate deeper in this experiment
        y_grasp_offset = 0.0125  # The distance from the screw hole to the center of the plate
            
        if self.assembly_database.db_name == "wrs_assembly_2021":
            if panel_name == "panel_bearing":
                offset_y = -0.0015             # MAGIC NUMBER (points to the global forward (x-axis))
                offset_z = -0.002            # MAGIC NUMBER (points to the global left (negative y-axis))(+ l_plate/2)
            else:  # panel_motor
                offset_y = -0.002             # MAGIC NUMBER
                offset_z = -0.003           # MAGIC NUMBER (+ l_plate/2)
        rotation_offset = np.deg2rad([180, 0, 0]).tolist()  # Robots are not perfectly flat in the table
        place_pose = conversions.to_pose_stamped(object_frame, [offset_height, y_grasp_offset + offset_y, l_plate/2 + offset_z] + rotation_offset)
        return place_pose
    
    def get_panel_grasp_pose(self, panel_name):
        if panel_name == "panel_bearing":
            grasp_pose = copy.deepcopy(self.assembly_database.get_grasp_pose(panel_name, "grasp_7"))
        elif panel_name == "panel_motor":
            grasp_pose = copy.deepcopy(self.assembly_database.get_grasp_pose(panel_name, "grasp_9"))

        if not grasp_pose:
            rospy.logerr("Could not load grasp pose " + "default_grasp" + " for object " + panel_name + ". Aborting pick.")
            return False
        return grasp_pose

    def grasp_panel_from_tray_upright(self, panel_name="panel_bearing"):
        #self.a_bot.go_to_named_pose("above_tray", speed=.5, acceleration=.3)
        grasp_pose = self.get_panel_grasp_pose(panel_name)
        grasp_pose = helpers.rotatePoseByRPY(-135, 0, 0, grasp_pose)
        grasp_pose.header.frame_id = "move_group/" + panel_name
        grasp_pose_tray = self.transform_pose_with_wait(grasp_pose)
        if panel_name == "panel_bearing":
            l = 0.116
            grasp_depth_offset = 0.04
        elif panel_name == "panel_motor":
            l = 0.06
            grasp_depth_offset = 0.03
        
        grasp_pose_tray.pose.position.z -= grasp_depth_offset

        # 1. Grasp
        self.simple_pick("a_bot", grasp_pose_tray, axis="z", grasp_width=0.13, lift_up_after_pick=False, speed_fast=1.0, speed_slow=0.3,
                            approach_height=0.08, approach_with_move_lin=True)
        rospy.sleep(0.5)
        self.a_bot.gripper.open(velocity=1.0, opening_width=0.05, wait=True)

        # 2. Push
        if panel_name == "panel_bearing":
            start_push_distance = 0.03
            extra_push_distance = 0.04  # The distance the plate is pushed past its original (assumed) position
        elif panel_name == "panel_motor":
            start_push_distance = 0.02
            extra_push_distance = 0.03  # The distance the plate is pushed past its original (assumed) position
        gripper_width_half = 0.01

        push_start_pose = copy.deepcopy(grasp_pose)
        push_start_pose.pose.position.x += l/2 + gripper_width_half + start_push_distance
        
        push_end_pose = copy.deepcopy(push_start_pose)
        push_end_pose.pose.position.x -= start_push_distance + extra_push_distance

        push_start_pose_tray = self.transform_pose_with_wait(push_start_pose)
        push_end_pose_tray = self.transform_pose_with_wait(push_end_pose)
        
        if panel_name == "panel_bearing":
            push_start_pose_tray.pose.position.z -= 0.05
            push_end_pose_tray.pose.position.z -= 0.05
        elif panel_name == "panel_motor":
            push_start_pose_tray.pose.position.z -= 0.035
            push_end_pose_tray.pose.position.z -= 0.035

        seq = []
        seq.append(helpers.to_sequence_item(push_start_pose_tray, linear=True))
        seq.append(helpers.to_sequence_gripper("close", gripper_force=0, gripper_velocity=1.0))
        seq.append(helpers.to_sequence_item(push_end_pose_tray, speed=0.04, linear=True))
        seq.append(helpers.to_sequence_item_relative([0,-0.001,0,0,0,0], speed=0.04))
        seq.append(helpers.to_sequence_gripper("open", gripper_velocity=0.003, gripper_opening_width=0.05, wait=True))
        # self.a_bot.move_lin(push_start_pose_tray)
        # self.a_bot.gripper.close()
        # self.a_bot.move_lin(push_end_pose_tray, speed=0.015)
        # self.a_bot.gripper.open(opening_width=0.06)
        self.execute_sequence("a_bot", seq, "center_panel_in_tray", plan_while_moving=True)
        
        # 3. Pick for placement
        grasp_pose = self.get_panel_grasp_pose(panel_name)
        grasp_pose = helpers.rotatePoseByRPY(-135, 0, 0, grasp_pose)
        grasp_pose.header.frame_id = "move_group/" + panel_name
        grasp_pose.pose.position.x -= extra_push_distance
        grasp_pose_tray = self.transform_pose_with_wait(grasp_pose)
        grasp_pose_tray.pose.position.z -= grasp_depth_offset

        self.simple_pick("a_bot", grasp_pose_tray, axis="z", grasp_width=0.06, minimum_grasp_width=0.0001, 
                            approach_height=0.0, speed_fast=0.4, speed_slow=0.2, retreat_height=0.1, approach_with_move_lin=True)
        return
    
    def place_panel_ihpe(self, panel_name="panel_bearing"):
        place_pose = self.get_panel_place_pose(panel_name)
        place_pose.pose.position.x += 0.001
        print("place pose",place_pose.pose)
        above_place_pose = copy.deepcopy(place_pose)
        above_place_pose.pose.position.x = -0.100

        self.planning_scene_interface.allow_collisions("base_fixture_top", panel_name)
        
        use_sequence = True
        if use_sequence:
            seq = []
            seq.append(helpers.to_sequence_item(above_place_pose, speed=0.8))
            seq.append(helpers.to_sequence_item(place_pose, speed=0.4))
            seq.append(helpers.to_sequence_gripper("open", gripper_velocity=0.001, gripper_opening_width=0.02, wait=True))
            seq.append(helpers.to_sequence_item_relative([0, -0.10, 0.0, 0, 0, 0]))
            self.execute_sequence("a_bot", seq, "place_panel_on_base", plan_while_moving=True)
            self.planning_scene_interface.disallow_collisions("base_fixture_top", panel_name)
            self.a_bot.gripper.forget_attached_item()
        else:
            pass
            # if not self.a_bot.go_to_pose_goal(above_place_pose, speed=0.5, move_lin=True, timeout=15):
            #     return False
            # self.a_bot.go_to_pose_goal(place_pose, speed=0.1, move_lin=True)
            # self.planning_scene_interface.disallow_collisions("base_fixture_top", panel_name)
            # self.a_bot.gripper.open(opening_width=0.02, velocity=0.01)
            # self.a_bot.gripper.forget_attached_item()

            # away_pose = copy.deepcopy(place_pose)
            # away_pose.pose.position.z += 0.150
            # self.a_bot.go_to_pose_goal(away_pose, speed=0.2)

        self.despawn_object(panel_name)
    
    def pick_panel_from_base(self, panel_name="panel_bearing", approach_height=0.08):
        place_pose = self.get_panel_place_pose(panel_name)
        
        self.simple_pick("a_bot", place_pose, axis="x", sign=-1, grasp_width=0.04, speed_fast=0.4, speed_slow=0.1,
                            approach_height=approach_height, retreat_height=0.08, approach_with_move_lin=True)

    def look_at_hole(self, robot_name="b_bot", panel_name="panel_bearing", log_image_name=""):
        self.activate_led(robot_name, on=True)
        self.vision.activate_camera(robot_name + "_outside_camera")
        robot = self.active_robots.get(robot_name)
        if panel_name == "panel_bearing":
            look_pose = conversions.to_pose_stamped("assembled_part_01_screw_hole_panel1_1", 
                                            [-0.145, 0.000, 0.006] + [0.033, -0.076, -0.010])
        elif panel_name == "panel_motor":
            look_pose = conversions.to_pose_stamped("assembled_part_01_screw_hole_panel2_1", 
                                            [-0.145, 0.000, 0.006] + [0.033, -0.076, -0.010])
        robot.go_to_pose_goal(look_pose, end_effector_link=robot_name + "_outside_camera_color_frame", 
                                    speed=.6, acceleration=.4, move_lin=True)
        # robot.move_joints(joint_pose_goal=[0.9144, -1.8169, 2.2137, -1.8209, -1.68, -0.6178])
        print(robot_name,np.round(robot.robot_group.get_current_joint_values(), 4).tolist())                                
        if log_image_name:
            self.write_cam_image(log_image_name)
        
        rospy.sleep(1.0)
        robot.go_to_named_pose("home")
        return

    # def look_at_hole(self, panel_name="panel_bearing", log_image_name=""):
    #     self.activate_led("b_bot", on=True)
    #     self.vision.activate_camera("b_bot_outside_camera")
    #     if panel_name == "panel_bearing":
    #         look_pose = conversions.to_pose_stamped("assembled_part_01_screw_hole_panel1_1", 
    #                                         [-0.145, 0.000, 0.006] + [0.033, -0.076, -0.010])
    #     elif panel_name == "panel_motor":
    #         look_pose = conversions.to_pose_stamped("assembled_part_01_screw_hole_panel2_1", 
    #                                         [-0.145, 0.000, 0.006] + [0.033, -0.076, -0.010])
    #     self.b_bot.go_to_pose_goal(look_pose, end_effector_link="b_bot_outside_camera_color_frame", 
    #                                 speed=.6, acceleration=.4, move_lin=True)
    #     # self.b_bot.move_joints(joint_pose_goal=[0.9144, -1.8169, 2.2137, -1.8209, -1.68, -0.6178])
    #     print("b_bot",np.round(self.b_bot.robot_group.get_current_joint_values(), 4).tolist())                                
    #     if log_image_name:
    #         self.write_cam_image(log_image_name)
        
    #     rospy.sleep(1.0)
    #     self.b_bot.go_to_named_pose("home")
    #     return
        
    def pick_bearing_ihpe(self, target_pose=None, grasp_width=0.08):
        """ If bearing_is_in_storage, the bearing is picked from storage and not the tray.
        """

        if not target_pose:
            goal = conversions.to_pose_stamped("tray_center", [0.01, 0.01, 0.02] + np.deg2rad([0, 90., 0]).tolist())
        else:
            goal = target_pose

        bearing_pose = copy.deepcopy(goal)
        bearing_pose.pose.position.z -= 0.01
        self.markers_scene.spawn_item("bearing", bearing_pose)
        # self.spawn_object("bearing", bearing_pose)
        self.planning_scene_interface.allow_collisions("bearing", "")

        robot_name = "a_bot"
        if not self.simple_pick(robot_name, goal, gripper_force=50.0, grasp_width=grasp_width,
                                axis="z", grasp_height=0.002, item_id_to_attach="bearing",
                                minimum_grasp_width=0.01, allow_collision_with_tray=True, approach_with_move_lin=True):
            rospy.logerr("Fail to pick bearing from tray")
            return False

    def return_bearing_to_tray(self, add_random_offset):
        # self.a_bot.move_lin_rel(relative_translation=[0.05, 0, 0], acceleration=0.25, speed=.5)
        self.allow_collisions_with_robot_hand("tray", "a_bot", True)
        rotation_offset = -1
        above_tray = conversions.to_pose_stamped("tray_center", [0, 0, 0.15, 0, tau/4, rotation_offset*tau/4])
        tray_center_pose = conversions.to_pose_stamped("tray_center", [0, 0, 0.025, 0, tau/4+radians(35), rotation_offset*tau/4])
        #tray_center_pose_vertical = conversions.to_pose_stamped("tray_center", [0, 0, 0.02, 0, tau/4, rotation_offset*tau/4])
        drop_pose = self.listener.transformPose("tray_center", tray_center_pose)
        #vertical_pose = self.listener.transformPose("tray_center", tray_center_pose_vertical)
        if add_random_offset:
            drop_pose.pose.position.x = np.random.uniform(low=-0.06, high=0.06)
            drop_pose.pose.position.y = np.random.uniform(low=-0.06, high=0.06)

        if not self.a_bot.go_to_pose_goal(above_tray, move_lin=True):
            rospy.logerr("fail to go to above_tray (drop_in_tray)")
            return False

        if not self.a_bot.go_to_pose_goal(drop_pose, move_lin=True):
            rospy.logerr("fail to go to drop_pose (drop_in_tray)")
            return False
        
        self.a_bot.gripper.open(opening_width=0.06)

        self.a_bot.move_lin_rel(relative_translation=[0,0,0], relative_rotation=[-radians(35),0,0])
        # vertical_angle = 90
        # vertical_pose = helpers.rotatePoseByRPY(0,vertical_angle, -vertical_angle, drop_pose)
        
        # if not self.a_bot.go_to_pose_goal(vertical_pose, move_lin=True):
        #     rospy.logerr("fail to go to drop_pose (drop_in_tray)")
        #     return False

        #drop_pose_tray = self.drop_in_tray("a_bot", add_random_offset)
        drop_pose_tray = copy.deepcopy(drop_pose)
        drop_pose_tray.pose.position.z = 0.01
        drop_pose_tray.pose.position.y -= 0.01
        grasp_pose_tray = copy.deepcopy(drop_pose_tray)

        if add_random_offset:
            angle_range = 45
            xy_range = 0.01

            offset_angle = np.random.uniform(radians(-angle_range), radians(angle_range))
            offset_x = np.random.uniform(-xy_range, xy_range)
            offset_y = np.random.uniform(-xy_range, xy_range)
            grasp_pose_tray = helpers.rotatePoseByRPY(offset_angle, -radians(35), 0, grasp_pose_tray)
            grasp_pose_tray.pose.position.x += offset_x
            grasp_pose_tray.pose.position.y += offset_y

        # above_drop_pose = copy.deepcopy(drop_pose_tray)
        # above_drop_pose.pose.position.z += 0.1
        # self.a_bot.move_lin(above_drop_pose, speed=0.5)
        # self.a_bot.move_lin(drop_pose_tray, speed=0.2)
        # self.a_bot.gripper.open(opening_width=0.06)
        # self.a_bot.move_lin(above_drop_pose, speed=0.5)
        self.allow_collisions_with_robot_hand("tray", "a_bot", False)
        if add_random_offset:
            return [offset_angle, offset_x, offset_y], grasp_pose_tray

    def pick_shaft_ihpe(self, target_pose=None):
        """ If shaft_is_in_storage, the shaft is picked from storage and not the tray.
        """

        if not target_pose:
            goal = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.0] + np.deg2rad([0, 90., 0]).tolist())
        else:
            goal = target_pose

        shaft_pose = copy.deepcopy(goal)
        shaft_pose.pose.position.z = 0.002
        # self.markers_scene.spawn_item("shaft", shaft_pose)
        gp = conversions.from_pose_to_list(goal.pose)
        gp[2] = 0.005
        rotation = transformations.euler_from_quaternion(gp[3:])[0] % (tau/2)
        print("gripper orientation from SSD", degrees(rotation))
        translation_correction = 0.075/2.0
        gp[:2] += [translation_correction*cos(rotation)-0.01, -translation_correction*sin(rotation)]  # Magic Numbers for visuals
        shaft_pose = conversions.to_pose_stamped("tray_center", gp[:3].tolist() + [0, 0, tau/2 - rotation])
        self.markers_scene.spawn_item("shaft", shaft_pose)
        # self.spawn_object("shaft", shaft_pose)
        self.planning_scene_interface.allow_collisions("shaft", "")

        robot_name = "b_bot"
        if not self.simple_pick(robot_name, goal, gripper_force=100.0, grasp_width=.085, approach_height=0.1, grasp_height=0,
                                    item_id_to_attach="shaft", minimum_grasp_width=0.003,  axis="z", lift_up_after_pick=True,
                                    speed_slow=0.5, allow_collision_with_tray=True, approach_with_move_lin=True):
            rospy.logerr("Fail to pick shaft from tray")
            return False
        return True


    def return_shaft_to_tray(self, add_random_offset):
        # self.a_bot.move_lin_rel(relative_translation=[0.05, 0, 0], acceleration=0.25, speed=.5)
        self.allow_collisions_with_robot_hand("tray", "b_bot", True)
        above_tray = conversions.to_pose_stamped("tray_center", [0, 0, 0.15, 0, tau/4, 0])
        tray_center_pose = conversions.to_pose_stamped("tray_center", [0, 0, 0.005, 0, tau/4, 0])
        
        drop_pose = self.listener.transformPose("tray_center", tray_center_pose)
        
        if add_random_offset:
            drop_pose.pose.position.x = np.random.uniform(low=-0.06, high=0.06)
            drop_pose.pose.position.y = np.random.uniform(low=-0.06, high=0.06)

        if not self.b_bot.go_to_pose_goal(above_tray, move_lin=True):
            rospy.logerr("fail to go to above_tray (drop_in_tray)")
            return False

        if not self.b_bot.go_to_pose_goal(drop_pose, move_lin=True):
            rospy.logerr("fail to go to drop_pose (drop_in_tray)")
            return False
        
        self.b_bot.gripper.open(opening_width=0.06)

        grasp_pose_tray = copy.deepcopy(drop_pose)

        if add_random_offset:
            angle_range = 30
            xy_range = 0.01

            offset_angle = np.random.uniform(radians(-angle_range), radians(angle_range))
            offset_x = np.random.uniform(-xy_range, xy_range)
            offset_y = np.random.uniform(-xy_range, xy_range)
            grasp_pose_tray = helpers.rotatePoseByRPY(offset_angle, 0, 0, grasp_pose_tray)
            grasp_pose_tray.pose.position.x += offset_x
            grasp_pose_tray.pose.position.y += offset_y
            grasp_pose_tray.pose.position.z -= 0.005

        self.allow_collisions_with_robot_hand("tray", "b_bot", False)
        if add_random_offset:
            return [offset_angle, offset_x, offset_y], grasp_pose_tray

    def look_at_shaft(self, log_image_name=""):
        rotation = [0, radians(-35.0), 0]
        pose = conversions.to_pose_stamped("assembled_part_07_inserted", [-0.016, 0.015, -0.003] + rotation)
        pose2 = conversions.to_pose_stamped("assembled_part_07_inserted", [-0.001, -0.019, 0.029, -0.008, -0.216, -0.001, 0.976])

        self.a_bot.go_to_pose_goal(pose, speed=0.2, move_lin=True)
        self.a_bot.go_to_pose_goal(pose2, speed=0.2, move_lin=True)
        self.vision.activate_camera("a_bot_inside_camera")
        self.activate_led("a_bot", True)
        # self.confirm_to_proceed("take picture")
        rospy.sleep(1)
        if log_image_name:
            self.write_cam_image(log_image_name)
        self.a_bot.move_lin_rel(relative_translation=[0,0,0.1])
        self.a_bot.go_to_named_pose("home")

    def look_at_bearing_housing(self, log_image_name=""):
        self.vision.activate_camera("extra_camera")
        if log_image_name:
            self.write_cam_image(log_image_name)

if __name__ == '__main__':
    try:
        rospy.init_node('o2ac_routines', anonymous=False)
        c = IHPEExperiment2021()
        panel_name = "panel_motor"
        while True:
            rospy.loginfo("Enter 1 to move the robots home.")
            rospy.loginfo("Enter 11,12 to close,open grippers")
            rospy.loginfo("Enter 15 (16) to equip (unequip) m4 tool (b_bot).")
            rospy.loginfo("Enter 4 to pick bearing panel.")
            rospy.loginfo("Enter 5 to place bearing panel.")
            rospy.loginfo("Enter 6 to look at hole with b_bot.")
            rospy.loginfo("Enter 7 to return panel from base, 7r without picking it.")
            rospy.loginfo("Enter reset to reset the scene")
            rospy.loginfo("Enter START to start the task.")
            rospy.loginfo("Enter x to exit.")
            i = input()
            tic_start = time.time()
            if i == '1':
                c.a_bot.go_to_named_pose("home")
                c.b_bot.go_to_named_pose("home")
            elif i == '1a':
                c.a_bot.go_to_named_pose("home")
            elif i == '1b':
                c.b_bot.go_to_named_pose("home")
            elif i == '11':
                c.a_bot.gripper.close(wait=False)
                c.b_bot.gripper.close()
            elif i == '12':
                c.a_bot.gripper.open(wait=False)
                c.b_bot.gripper.open()
            elif i == '15':
                c.do_change_tool_action("b_bot", equip=True, screw_size=4)
            elif i == '16':
                c.do_change_tool_action("b_bot", equip=False, screw_size=4)
            elif i == '32':
                c.pick_screw_from_feeder("b_bot", screw_size=4)
            elif i == "setbearing":
                panel_name = "panel_bearing"
                c.reset_scene_and_robots()
                c.reset_assembly_visualization()
            elif i == "setmotor":
                panel_name = "panel_motor"
                c.reset_scene_and_robots()
                c.reset_assembly_visualization()
            elif i == "4":
                c.spawn_panel(panel_name=panel_name)
                rospy.sleep(1.0)
                c.grasp_panel_from_tray_upright(panel_name=panel_name)
            elif i == "5":
                c.place_panel_ihpe(panel_name=panel_name)
            elif i == "6":
                c.look_at_hole(panel_name=panel_name)
            elif i == "6t":
                c.look_at_hole(panel_name=panel_name, log_image_name="test")
            elif i == "7":
                c.pick_panel_from_base(panel_name=panel_name)
                c.return_panel_to_tray(panel_name=panel_name, add_random_offset=True)
            elif i == "7r":
                c.return_panel_to_tray(panel_name=panel_name, add_random_offset=True)
            elif i == "7b":
                c.pick_panel_from_base(panel_name=panel_name)
                c.return_panel_to_tray(panel_name=panel_name, add_random_offset=False)
            elif i == "look_hole":
                c.look_at_hole(robot_name="a_bot")
            elif i == "cam":
                c.write_cam_image()
            elif i == "camwritetest":
                c.write_cam_image("test.jpg")
            elif i == "cam_shaft":
                c.look_at_shaft()
            elif i == "fasten":
                def dummy_Task(): 
                    c.a_bot.go_to_named_pose("home")
                    return True
                if not c.fasten_panel("panel_bearing", simultaneous=True, a_bot_task_2nd_screw=dummy_Task):
                    rospy.logerr("failed")
            if i == "fullrun":
                # Requires the plate to be on the base (execute 4, 5 first)
                starting_offsets = []
                # for i in range(102):
                # for i in [100,101,102,103]:
                for i in range(1):
                    c.pick_panel_from_base(panel_name=panel_name, approach_height=0.02)
                    panel_place_offsets = c.return_panel_to_tray(panel_name=panel_name, add_random_offset=True)
                    starting_offsets.append(panel_place_offsets)
                    print("Running test nr. " + str(i+1) + " with offsets:")
                    print(panel_place_offsets)
                    print("Current log:")
                    print(starting_offsets)
                    c.spawn_panel(panel_name=panel_name)
                    rospy.sleep(0.1)
                    c.grasp_panel_from_tray_upright(panel_name=panel_name)
                    if c.a_bot.is_protective_stopped():
                        c.confirm_to_proceed("Oops. Got stuck while picking. Please clean up, release protective stop and press enter to proceed.")
                    c.place_panel_ihpe(panel_name=panel_name, )
                    if c.a_bot.is_protective_stopped():
                        c.confirm_to_proceed("Oops. Got stuck while placing. Please clean up, release protective stop and press enter to proceed.")
                    # c.look_at_hole(panel_name=panel_name, log_image_name=panel_name + "_view" + str(i+1))
                
                print("==================================")
                print("Experiment log (starting_offsets [offset_angle, offset_x, offset_y]):")
                print(starting_offsets)

            if i == "fullrun_bearing" or i == "456":
                starting_offsets = []
                # grasp from tray center
                c.a_bot.go_to_named_pose("above_tray", speed=.5, acceleration=.3)
                c.use_dummy_vision = True
                c.pick_bearing_ihpe(target_pose=None, grasp_width=0.1)
                c.orient_bearing("assembly", "a_bot")

                for i in range(3):
                    c.a_bot.go_to_named_pose("above_tray", speed=.5, acceleration=.3)
                    place_offsets, grasp_pose = c.return_bearing_to_tray(add_random_offset=True)
                    
                    starting_offsets.append(place_offsets)
                    print("Running test nr. " + str(i+1) + " with offsets:")
                    print(place_offsets)
                    print("Current log:")
                    print(starting_offsets)
                    
                    # c.confirm_to_proceed("pick again---")
                    c.pick_bearing_ihpe(target_pose=grasp_pose)
                    # c.confirm_to_proceed("orient---")
                    if c.a_bot.is_protective_stopped():
                        c.confirm_to_proceed("Oops. Got stuck while picking. Please clean up, release protective stop and press enter to proceed.")
                    if not c.orient_bearing("assembly", "a_bot"):
                        break
                    if c.a_bot.is_protective_stopped():
                        c.confirm_to_proceed("Oops. Got stuck while placing. Please clean up, release protective stop and press enter to proceed.")
                    # c.look_at_hole(panel_name=panel_name, log_image_name=panel_name + "_view" + str(i+1))
                    rospy.sleep(2)
                    c.look_at_bearing_housing(log_image_name="bearing_view_" + str(i+1))
                
                print("==================================")
                print("Experiment log (starting_offsets [offset_angle, offset_x, offset_y]):")
                print(starting_offsets)

            if i == "fullrun_shaft" or i == "123":
                starting_offsets = []
                # grasp from tray center
                c.b_bot.go_to_named_pose("above_tray", speed=.5, acceleration=.3)
                c.use_dummy_vision = True
                c.pick_shaft_ihpe(target_pose=None)

                c.confirm_to_proceed("Start experiment")
                for i in range(51,120):
                    c.b_bot.go_to_named_pose("above_tray", speed=.5, acceleration=.3)
                    place_offsets, grasp_pose = c.return_shaft_to_tray(add_random_offset=True)
                    
                    starting_offsets.append(place_offsets)
                    print("Running test nr. " + str(i+1) + " with offsets:")
                    print(place_offsets)
                    print("Current log:")
                    print(starting_offsets)
                    
                    # c.confirm_to_proceed("pick again---")
                    if not c.pick_shaft_ihpe(target_pose=grasp_pose):
                        break
                    # c.confirm_to_proceed("Center shaft")
                    # c.confirm_to_proceed("orient---")
                    if c.b_bot.is_protective_stopped():
                        c.confirm_to_proceed("Oops. Got stuck while picking. Please clean up, release protective stop and press enter to proceed.")
                    c.centering_shaft()
                    if not c.align_shaft("assembled_part_07_inserted", pre_insert_offset=0.055):
                        break
                    if c.b_bot.is_protective_stopped():
                        c.confirm_to_proceed("Oops. Got stuck while placing. Please clean up, release protective stop and press enter to proceed.")

                    c.look_at_shaft(log_image_name="shaft_view_" + str(i+1))

                    c.b_bot.move_lin_rel(relative_translation=[-0.05, 0, 0])
                    c.b_bot.move_lin_rel(relative_translation=[0, 0, 0.2])
                
                print("==================================")
                print("Experiment log (starting_offsets [offset_angle, offset_x, offset_y]):")
                print(starting_offsets)

            if i == "reset":
                c.reset_scene_and_robots()
                c.reset_assembly_visualization()
            if i == "activate":
                c.a_bot.activate_ros_control_on_ur()
                c.b_bot.activate_ros_control_on_ur()
            elif i == 'x':
                break
            elif i == "":
                continue
            print("This took: %.3f seconds" % (time.time() - tic_start))
    except rospy.ROSInterruptException:
        pass
