#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2022, OMRON SINIC X
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
# Author: Cristian C. Beltran-Hernandez, Remi S. Paillaud-Iwabuchi


from datetime import datetime
from math import pi
import signal

import numpy as np
from o2ac_routines.cooking import Cooking
from ur_control import conversions
import time
import sys
import rospy

import matplotlib.pyplot as plt


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def collect_dataset_manual_cutting(self: Cooking):
    # ps = conversions.to_pose_stamped("cutting_board", [0.04, 0, 0.42, 0.000, 0.670, 0.001, 0.742])
    # self.a_bot.go_to_pose_goal(ps, end_effector_link="a_bot_outside_camera_color_frame", speed=.5, acceleration=.3, wait=True, move_lin=True)

    i = 1
    while True:
        rospy.loginfo("==========================================")
        rospy.loginfo("Press any key to take picture INITIAL STATE: #%d" % i)
        input()
        self.write_cam_image("%d_initial_state" % i)
        rospy.loginfo("==========================================")
        rospy.loginfo("Press any key to take picture FINAL STATE: #%d" % i)
        input()
        self.write_cam_image("%d_final_state" % i)
        i += 1


def slice_vegetable_at_fix_velocity2(self: Cooking, cuts=5):
    distance_z = 0.06
    initial_y = 0.0

    slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, initial_y, distance_z, -pi, pi/2, 0])
    self.b_bot.go_to_pose_goal(slicing_pose, speed=0.2, end_effector_link=self.knife_center_frame, move_lin=True)
    self.confirm_to_proceed("start")

    y_offset = 0
    speed = 0.010
    for i in range(cuts):
        y_offset = i * 0.00 # move 5mm to the left
        init_pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, initial_y - y_offset, distance_z, -pi, pi/2, 0])
        self.b_bot.go_to_pose_goal(init_pose, speed=0.2, end_effector_link=self.knife_center_frame, move_lin=True)
        self.confirm_to_proceed("cut")

        target_pose = self.b_bot.move_lin_rel(relative_translation=[0, 0, -(distance_z + 0.003)], pose_only=True)
        target_pose = self.listener.transformPose("world", target_pose).pose
        self.b_bot.robot_group.limit_max_cartesian_link_speed(speed=speed, link_name="b_bot_gripper_tip_link")
        trajectory, _ = self.b_bot.robot_group.compute_cartesian_path([target_pose], eef_step=0.001, jump_threshold=3.0)
        self.b_bot.robot_group.clear_max_cartesian_link_speed()
        self.b_bot.force_controller.zero_ft_sensor()
        self.b_bot.robot_group.execute(trajectory, wait=False)

        duration = distance_z/speed + 2
        record_force_profile(self, duration=duration)

        self.b_bot.go_to_pose_goal(init_pose, speed=0.2, end_effector_link=self.knife_center_frame, move_lin=True)


def slice_vegetable_at_fix_velocity(self: Cooking):

    slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.01, 0.03, -pi, pi/2, 0])
    self.b_bot.go_to_pose_goal(slicing_pose, speed=0.2, end_effector_link=self.knife_center_frame, move_lin=True)
    self.confirm_to_proceed("cut")
    # move_down(self, num_of_points=10, distance=0.05, velocity=0.001)  # 5cm at 1 mm/s

    speed = 0.005

    target_pose = self.b_bot.move_lin_rel(relative_translation=[0.01, 0, -0.033], pose_only=True)
    target_pose = self.listener.transformPose("world", target_pose).pose
    self.b_bot.robot_group.limit_max_cartesian_link_speed(speed=speed, link_name="b_bot_gripper_tip_link")
    trajectory, _ = self.b_bot.robot_group.compute_cartesian_path([target_pose], eef_step=0.001, jump_threshold=3.0)
    # print("res", type(trajectory), trajectory)
    self.b_bot.robot_group.clear_max_cartesian_link_speed()
    self.b_bot.force_controller.zero_ft_sensor()
    self.b_bot.robot_group.execute(trajectory, wait=False)

    duration = 0.05/speed + 1
    record_force_profile(self, duration=duration)

    self.b_bot.go_to_pose_goal(slicing_pose, speed=0.2, end_effector_link=self.knife_center_frame, move_lin=True)


def record_force_profile(self: Cooking, duration=5, plot=False):

    ft_sensor_data = []
    wrench_data = []

    robot = self.b_bot.force_controller
    robot._init_ik_solver(base_link=robot.base_link, ee_link=self.knife_center_frame)
    robot.set_ft_filtering(active=False)
    init_time = rospy.get_time()
    r = rospy.Rate(500)
    while rospy.get_time() - init_time < duration:
        ft_sensor_data.append(robot.get_ee_wrench().tolist() + [rospy.get_time() - init_time])
        wrench_data.append(robot.get_ee_wrench(hand_frame_control=True).tolist() + [rospy.get_time() - init_time])
        r.sleep()
    rospy.loginfo(" =====  Finished ==== ")
    robot._init_ik_solver(base_link=robot.base_link, ee_link=robot.ee_link)

    now = datetime.now()
    timesuffix = now.strftime("%Y-%m-%d_%H-%M-%S")
    folder = "/root/o2ac-ur/catkin_ws/src/o2ac_routines/log/"
    np.save(folder + "ft_sensor-" + timesuffix, np.array(ft_sensor_data))
    np.save(folder + "wrench-" + timesuffix, np.array(wrench_data))

    if plot:
        time = np.linspace(0, duration, len(wrench_data))
        for i, ax in enumerate(["X", "Y", "Z"]):
            plt.plot(time, np.array(wrench_data)[:, i], label=ax)
        plt.ylim((-50,10))
        plt.legend()
        plt.show()


def move_down(self: Cooking, num_of_points, distance, velocity):
    robot = self.b_bot.force_controller
    trajectory_points = []
    trajectory_vel = []
    initial_pose = robot.end_effector()
    previous_joint_q = robot.joint_angles()

    dt = (distance/num_of_points)/velocity
    for i in range(num_of_points):
        # compute intermediate point
        midway_point = np.copy(initial_pose)
        midway_point[2] -= (distance/num_of_points)*i
        # compute ik
        joint_target = robot._solve_ik(midway_point, q_guess=previous_joint_q.tolist(), verbose=False)

        if joint_target is None:
            rospy.logerr("Failed to create trajectory")
            return

        joint_velocity = np.abs(joint_target - previous_joint_q) / dt

        # Append point
        trajectory_points.append(joint_target)
        trajectory_vel.append(joint_velocity)

        previous_joint_q = np.copy(joint_target)

    duration = distance/velocity
    robot.set_joint_trajectory(trajectory_points, velocities=trajectory_vel, t=duration)


if __name__ == '__main__':
    try:
        rospy.init_node('o2ac_routines', anonymous=False)
        rospy.loginfo(" ...")
        c = Cooking()
        while True:
            rospy.loginfo("Enter 1 for both robots to go home.")
            rospy.loginfo("Enter 1a for a bot to go home.")
            rospy.loginfo("Enter 1b for b bot to go home.")
            rospy.loginfo("Enter 2 to collect dataset from tray.")
            rospy.loginfo("Enter 3 to collect dataset from cooking board")
            rospy.loginfo("Enter 4 to collect dataset of manual cutting")
            rospy.loginfo("Enter x to exit.")
            i = input()
            tic_start = time.time()

            if i == '1':  # go home
                rospy.loginfo("Going home")
                c.ab_bot.go_to_named_pose("home")
            elif i == '1a':
                c.a_bot.go_to_named_pose("home")
            elif i == '1b':
                c.b_bot.go_to_named_pose("home")

            elif i == '2':  # Tray
                rospy.loginfo("Collecting dataset from tray")
                c.collect_dataset_tray()

            elif i == '3':  # cooking board
                rospy.loginfo("Collecting dataset from cooking board")
                c.collect_dataset_cutting_board()

            elif i == '4':
                collect_dataset_manual_cutting(c)

            #########################################
            ## Force profile of slicing vegetables ##
            #########################################

            # elif i == '10':  # Slice
            #     slice_vegetable_at_fix_velocity(c)
            elif i == '10b':  # Slice
                slice_vegetable_at_fix_velocity2(c)

            elif i == 'x':
                break
            elif i == "":
                continue
            print("This took: %.3f seconds" % (time.time() - tic_start))
    except rospy.ROSInterruptException:
        pass
