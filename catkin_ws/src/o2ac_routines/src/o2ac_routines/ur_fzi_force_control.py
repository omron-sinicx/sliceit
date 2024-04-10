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
# Author: Cristian C. Beltran-Hernandez

from o2ac_routines import helpers
import rospy
import rospkg
import yaml

import numpy as np

from ur_control.fzi_cartesian_compliance_controller import CompliantController
from ur_control import utils, traj_utils, conversions
from ur_control.constants import STOP_ON_TARGET_FORCE, TERMINATION_CRITERIA, DONE

from o2ac_routines.helpers import get_target_force, get_direction_index, get_orthogonal_plane, get_random_valid_direction


class URForceController(CompliantController):
    """ Compliant control functions """

    def __init__(self, robot_name, listener, tcp_link='gripper_tip_link', **kwargs):
        self.listener = listener
        self.default_tcp_link = robot_name + '_' + tcp_link
        self.robot_name = robot_name

        CompliantController.__init__(self, ft_topic='wrench', namespace=robot_name,
                                     joint_names_prefix=robot_name+'_', robot_urdf=robot_name,
                                     robot_urdf_package='o2ac_scene_description',
                                     ee_link=tcp_link, **kwargs)

        self.max_force_torque = [30., 30., 30., 4., 4., 4.]
        self.p_gains = [0.01, 0.01, 0.01, 1.0, 1.0, 1.0]

    def force_control(self, target_force=None, target_positions=None,
                      force_position_selection_matrix=None,
                      timeout=10.0, stop_on_target_force=False,
                      termination_criteria=None, end_effector_link=None):
        """ 
            Use with caution!! 
            target_force: list[6], target force for each direction x,y,z,ax,ay,az
            target_positions: array[array[7]] or array[7], can define a single target pose or a trajectory of multiple poses.
            force_position_selection_matrix: list[6], define which direction is controlled by position(1.0) or force(0.0)
            relative_to_ee: bool, whether to use the base_link of the robot as frame or the ee_link (+ ee_transform)
            timeout: float, duration in seconds of the force control
            termination_criteria: lambda, condition that if achieved stops the force control otherwise keep going until timeout
            displacement_epsilon: float, minimum translation displacement before considering the robot non-static
            check_displacement_time: float, time interval for checking whether the robot is static or not (see argument displacement_epsilon)
                                            The condition of static/non-static can be access when defining `termination_criteria`
        """
        if end_effector_link and end_effector_link != self.default_tcp_link:
            # Init IK and FK solvers with new end effector link
            self.set_end_effector_link(end_effector_link)

        self.zero_ft_sensor()  # offset the force sensor

        self.set_control_mode("parallel")
        if force_position_selection_matrix is not None:
            self.update_selection_matrix(force_position_selection_matrix)
        self.update_pd_gains(p_gains=self.p_gains)

        target_positions = self.end_effector() if target_positions is None else np.array(target_positions)
        target_force = np.array([0., 0., 0., 0., 0., 0.]) if target_force is None else np.array(target_force)

        result = self.execute_compliance_control(target_positions, target_force, max_force_torque=self.max_force_torque, duration=timeout,
                                                 stop_on_target_force=stop_on_target_force, termination_criteria=termination_criteria)

        if end_effector_link and end_effector_link != self.default_tcp_link:
            # Init IK and FK solvers with default end effector link
            self.set_end_effector_link(end_effector_link)

        return result

    def execute_circular_trajectory(self, *args, **kwargs):
        """
            Execute a circular trajectory on a given plane, with respect to the robot's end effector, with a given radius
            Note: we assume that the robot is in its starting position 
        """
        kwargs.update({"trajectory_type": "circular"})
        return self.execute_trajectory(*args, **kwargs)

    def execute_spiral_trajectory(self, *args, **kwargs):
        """
            Execute a spiral trajectory on a given plane, with respect to the robot's end effector, with a given radius
            Note: we assume that the robot is in its starting position 
        """
        kwargs.update({"trajectory_type": "spiral"})
        return self.execute_trajectory(*args, **kwargs)

    def execute_trajectory(self, plane, max_radius, radius_direction=None,
                           steps=100, revolutions=5,
                           wiggle_direction=None, wiggle_angle=0.0, wiggle_revolutions=0.0,
                           target_force=None, force_position_selection_matrix=None, timeout=10.,
                           termination_criteria=None, end_effector_link=None, trajectory_type="spiral"):
        from_center = True if trajectory_type == "spiral" else False
        eff = self.default_tcp_link if not end_effector_link else end_effector_link
        direction = radius_direction if radius_direction else helpers.get_random_valid_direction(plane)
        dummy_trajectory = traj_utils.compute_trajectory([0, 0, 0, 0, 0, 0, 1.],
                                                         plane, max_radius, direction, steps, revolutions,
                                                         from_center=from_center, trajectory_type=trajectory_type,
                                                         wiggle_direction=wiggle_direction, wiggle_angle=wiggle_angle,
                                                         wiggle_revolutions=wiggle_revolutions)
        # convert dummy_trajectory (initial pose frame id) to robot's base frame
        now = rospy.Time.now()
        try:
            self.listener.waitForTransform(self.base_link, eff, now, rospy.Duration(1))
            transform2target = self.listener.fromTranslationRotation(*self.listener.lookupTransform(self.base_link, eff, now))
        except:
            return False

        trajectory = []
        for p in dummy_trajectory:
            ps = conversions.to_pose_stamped(self.base_link, p)
            trajectory.append(conversions.from_pose_to_list(conversions.transform_pose(self.base_link, transform2target, ps).pose))

        sm = force_position_selection_matrix if force_position_selection_matrix else [1., 1., 1., 1., 1., 1.]  # no force control by default
        return self.force_control(target_force=target_force, target_positions=trajectory, force_position_selection_matrix=sm,
                                  timeout=timeout, termination_criteria=termination_criteria,
                                  end_effector_link=end_effector_link)

    def linear_push(self, force, direction, max_translation=None,
                    relative_to_ee=False, timeout=10.0, slow=False,
                    force_position_selection_matrix=None):
        """
        Apply force control in one direction until contact with `force`
        robot_name: string, name of the robot
        force: float, desired force
        direction: string, direction for linear_push +- X,Y,Z relative to base or end-effector, see next argument
        relative_to_ee: bool, whether to use the base_link of the robot as frame or the ee_link (+ ee_transform)
        """

        offset = 1 if "Z" in direction or self.robot_name == "b_bot" else -1
        target_force = np.array([0., 0., 0., 0., 0., 0.])
        sign = 1. if '+' in direction else -1.
        target_force[get_direction_index(direction[1])] = force * sign
        target_force *= offset

        if force_position_selection_matrix is None:
            force_position_selection_matrix = np.zeros(6)
            force_position_selection_matrix = np.array(target_force == 0.0) * 1.0  # define the selection matrix based on the target force

        initial_pose = self.end_effector()[get_direction_index(direction[1])]

        if max_translation is not None:
            def termination_criteria(current_pose): return abs(initial_pose - current_pose[get_direction_index(direction[1])]) >= max_translation
        else:
            termination_criteria = None

        result = self.force_control(target_force=target_force, force_position_selection_matrix=force_position_selection_matrix,
                                    timeout=timeout, stop_on_target_force=True, termination_criteria=termination_criteria)

        if result in (TERMINATION_CRITERIA, DONE, STOP_ON_TARGET_FORCE):
            rospy.loginfo("Completed linear_push: %s" % result)
            return True
        rospy.logerr("Fail to complete linear_push %s" % result)
        return False

    def do_insertion(self, target_pose_in_target_frame, insertion_direction, timeout,
                     radius=0.0, radius_direction=None, revolutions=3, force=1.0, goal_tolerance_if_lockup=0.0,
                     wiggle_direction=None, wiggle_angle=0.0, wiggle_revolutions=0.0,
                     force_position_selection_matrix=None):
        """
            target_pose_in_target_frame: PoseStamp, target in target frame
            insertion_direction: string, [+/-] "X", "Y", or "Z" in robot's base frame! Note: limited to one direction 
            TODO: convert to target frame? or compute from target frame?
            relaxed_target_by: float, distance to allow the insertion to succeed if there is no motion in 2 seconds
            TODO: try to make the insertion direction define-able as an axis with respect to the object-to-be-inserted and the target-frame
        """
        axis = get_direction_index(insertion_direction[1])
        plane = get_orthogonal_plane(insertion_direction[1])
        radius_direction = get_random_valid_direction(plane) if radius_direction is None else radius_direction

        offset = 1 if "Z" in insertion_direction or self.robot_name == "b_bot" else -1
        target_force = [0., 0., 0., 0., 0., 0.]
        sign = 1. if '+' in insertion_direction else -1.
        target_force[get_direction_index(insertion_direction[1])] = force * sign
        target_force *= offset

        if force_position_selection_matrix is None:
            force_position_selection_matrix = np.array(target_force == 0.0) * 0.8  # define the selection matrix based on the target force

        translation, rotation = self.listener.lookupTransform(target_pose_in_target_frame.header.frame_id, self.ns + "_base_link", rospy.Time.now())
        transform2target = self.listener.fromTranslationRotation(translation, rotation)

        start_pose_robot_base = conversions.to_pose_stamped(self.ns + "_base_link", self.end_effector())
        start_pose_in_target_frame = conversions.transform_pose(target_pose_in_target_frame.header.frame_id, transform2target, start_pose_robot_base)
        start_pose_of = conversions.from_pose_to_list(start_pose_in_target_frame.pose)
        target_pose_of = conversions.from_pose_to_list(target_pose_in_target_frame.pose)
        more_than = start_pose_of[axis] < target_pose_of[axis]

        def termination_criteria(current_pose):
            current_pose_robot_base = conversions.to_pose_stamped(self.ns + "_base_link", current_pose)
            current_pose_in_target_frame = conversions.transform_pose(target_pose_in_target_frame.header.frame_id, transform2target, current_pose_robot_base)
            current_pose_of = conversions.from_pose_to_list(current_pose_in_target_frame.pose)
            # print("check cp,tp", current_pose_of[axis], target_pose_of[axis])
            if more_than:
                return current_pose_of[axis] >= target_pose_of[axis] or \
                    (current_pose_of[axis] >= target_pose_of[axis] - goal_tolerance_if_lockup)
            return current_pose_of[axis] <= target_pose_of[axis] or \
                (current_pose_of[axis] <= target_pose_of[axis] + goal_tolerance_if_lockup)

        result = self.execute_spiral_trajectory(plane, max_radius=radius, radius_direction=radius_direction, steps=100, revolutions=revolutions,
                                                wiggle_direction=wiggle_direction, wiggle_angle=wiggle_angle, wiggle_revolutions=wiggle_revolutions,
                                                target_force=target_force, force_position_selection_matrix=force_position_selection_matrix, timeout=timeout,
                                                termination_criteria=termination_criteria)

        if result in (TERMINATION_CRITERIA, DONE, STOP_ON_TARGET_FORCE):
            rospy.loginfo("Completed insertion with state: %s" % result)
        else:
            rospy.logerr("Fail to complete insertion with state %s" % result)
        return result
