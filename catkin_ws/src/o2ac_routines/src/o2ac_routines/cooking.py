import sensor_msgs
import os
from datetime import datetime
import cv2
import cv_bridge  # This offers conversion methods between OpenCV
from o2ac_routines.common import O2ACCommon
import tf_conversions
from ur_control import conversions, traj_utils
import rospy
import geometry_msgs.msg
import copy
from math import pi, radians
import o2ac_routines.helpers as helpers

import numpy as np
tau = 2*pi


# For writing the camera image


class Cooking(O2ACCommon):
    def __init__(self):
        super(Cooking, self).__init__()
        self.assembly_database.change_assembly("cooking")

        # For recording the camera image
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera_multiplexer/image", sensor_msgs.msg.Image, self.image_callback)
        self.cam_image = []

        self.object_mesh_lengths = {"1": 0.192}

        if self.use_gazebo_sim:
            self.knife_center_frame = "b_bot_knife_center"
            self.planning_scene_interface.allow_collisions("base_fixture_top", "b_bot_right_inner_knuckle")
            self.planning_scene_interface.allow_collisions("cutting_board", "b_bot_right_inner_knuckle")
        else:
            self.knife_center_frame = "b_bot_knife_center"
        self.knife_equipped = False

        # dummy vision pose
        self.dummy_pose = [0.0, 0.05, 0.01] + np.deg2rad([0, 90., 0]).tolist()

        self.slicing_rl = SlicingRLPolicy(policy_directory="/root/o2ac-ur/results/SAC_slicing_3d-disect-gz")

    def load_objects(self):
        poses = [
            [0.23, 0.02, 0.02, 0, 0, 0],
            [0.15, 0.1, 0.02, 0, 0, 0],
            # [0.1, 0.03, 0.03, radians(90), 0, 0],
            # [0.15, -0.05, 0.03, 0, 0, 0],
            # [0.1, -0.08, 0.02, 0, 0, 0],
            # [0.27, 0.04, 0.02, 0, 0, 0],
            # [0.085, 0.15, 0.02, radians(0), 0, 0],
        ]
        # self.spawn_multiple_objects("cooking",["tomato","cucumber","eggplant","onion","banana", "corn", "artichoke"], poses, "workspace_center")
        self.spawn_multiple_objects(
            "cooking", ["tomato", "cucumber"], poses, "workspace_center")

    def get_object_info(self, object_id_or_name):
        obj_id = self.assembly_database.name_to_id(object_id_or_name)

        if not obj_id:
            obj_name = self.assembly_database.id_to_name(object_id_or_name)
        else:
            obj_name = self.assembly_database.id_to_name(obj_id)

        if not obj_name:
            rospy.logerr("Could not find object: " + object_id_or_name + " in database!")
            return -1, ""

        self.dummy_vision_objects = {
            "cucumber": conversions.to_pose_stamped("tray_center", [0.0, 0.05, 0.01] + np.deg2rad([0, 90., 0]).tolist()),
            "half_cucumber": conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.05, 0.01] + np.deg2rad([0, 90., 0]).tolist())
        }

        return obj_id, obj_name

# Vision

    def image_callback(self, msg):
        self.cam_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")

    def write_cam_image(self, image_name="b_bot_cam"):
        now = datetime.now()
        timesuffix = now.strftime("%Y-%m-%d_%H:%M:%S")
        folder = os.path.join(self.rospack.get_path(
            "o2ac_routines"), "log/cooking")
        if self.cam_image is not None:
            cv2.imwrite(os.path.join(folder, image_name + "_" +
                        timesuffix + ".png"), np.array(self.cam_image))
        else:
            rospy.logwarn("No image found")

    def visionary_load_object(self, object_poses):
        # {"obj_id": pose}
        poses = []
        object_names = []
        object_scales = []

        if object_poses.get(1, None):  # PoseStamp
            p = conversions.from_pose_to_list(object_poses.get(1, None).pose)
            p[2] = 0.005
            poses.append(p)
            object_names.append("cucumber")
            if self.object_mesh_lengths.get(1, None):
                object_scales.append()
        if object_poses.get(2, None):  # PoseStamp
            p = conversions.from_pose_to_list(object_poses.get(2, None).pose)
            p[2] = 0.0
            poses.append(p)
            object_names.append("tomato")
        self.spawn_multiple_objects(
            "cooking", object_names, poses, "tray_center")

    def look_and_get_object_pose(self, object_id, robot_name="b_bot", frame_id="tray_center", multiple_views=True):
        """
        Looks at the tray from above and gets grasp points of items.
        Does very light feasibility check before returning.
        """

        # Make sure object_id is the id number
        object_id, _ = self.get_object_info(object_id)

        self.vision.activate_camera(robot_name + "_outside_camera")
        self.activate_led(robot_name)
        if frame_id == "tray_center":
            self.active_robots[robot_name].go_to_named_pose("above_tray", speed=1.0)

        if object_id in self.objects_in_tray:
            del self.objects_in_tray[object_id]

        if self.use_dummy_vision or not (self.use_real_robot or self.use_gazebo_sim):
            rospy.logwarn(
                "Using dummy vision! Setting object pose to tray center.")
            self.objects_in_tray[object_id] = conversions.to_pose_stamped(frame_id, self.dummy_pose)
            return self.objects_in_tray[object_id]

        tray_view_high, close_tray_views = self.define_local_tray_views(robot_name=robot_name, include_rotated_views=multiple_views,
                                                                        high_height=0.42, low_height=0.42,
                                                                        frame_id=frame_id,
                                                                        offsets=[0.0, 0.05])

        if frame_id == "cutting_board":
            tray_view_high.pose.position.x -= 0.06

        if multiple_views:
            tray_views = [tray_view_high] + close_tray_views
        else:
            tray_views = [tray_view_high]

        for view in tray_views:
            assert not rospy.is_shutdown()
            self.vision.activate_camera(robot_name + "_outside_camera")
            print("view", view)

            self.active_robots[robot_name].go_to_pose_goal(
                view, end_effector_link=robot_name + "_outside_camera_color_frame", speed=.5, acceleration=.3, wait=True, move_lin=True)
            rospy.sleep(0.5)

            tries = 5
            while tries > 0:
                if self.get_3d_poses_from_ssd():
                    break
                rospy.sleep(0.3)
                tries -= 1

            object_pose = copy.deepcopy(
                self.objects_in_tray.get(object_id, None))
            if object_pose:
                return object_pose

        rospy.logerr("Could not find item id " + str(object_id) + " in tray!")
        return False

    def visualize_object(self, object_id, robot_name="a_bot"):
        obj_id = self.get_object_id(object_id)
        obj_name = self.assembly_database.id_to_name(obj_id)

        if not self.look_and_get_object_pose(object_id=obj_id, robot_name=robot_name, frame_id="tray_center", multiple_views=False):
            return False

        dims = self.object_in_tray_dimensions.get(obj_id)  # width, length

        if not dims:
            return False

        width, length = dims
        print("width = ", width)
        print("length = ", length)
        # TODO scale mesh using length
        if obj_name == "cucumber":
            print("hi")
            z_scale = length / 0.192 * 0.001
            xy_scale = width / 0.03 * 0.001
            print("scale", xy_scale, xy_scale, z_scale)  # TO BE CHECKED
            self.spawn_object(obj_name, self.objects_in_tray.get(obj_id), "tray_center", scale=(xy_scale, xy_scale, z_scale))
        else:
            self.visionary_load_object(self.objects_in_tray)

# Equip/Unequip tools

    def equip_knife(self):
        if self.b_bot.robot_status.carrying_tool:
            rospy.logwarn("Robot already holding a tool: %s" % self.b_bot.robot_status.held_tool_id)
            return

        self.spawn_tool("knife")
        self.b_bot.gripper.open()

        self.b_bot.go_to_named_pose("knife_pick_up_ready", speed=0.2)

        equip_pose = conversions.to_pose_stamped(
            "knife_pickup_link", [0.03, 0, 0, 0, 0, 0])

        self.b_bot.go_to_pose_goal(equip_pose, speed=.1, move_lin=True)
        self.confirm_to_proceed("fix pose")

        self.allow_collisions_with_robot_hand("knife", "b_bot")
        self.b_bot.gripper.close(velocity=0.03, force=100)
        self.b_bot.gripper.attach_object("knife", with_collisions=True)
        self.b_bot.gripper.forget_attached_item()
        self.b_bot.robot_status.carrying_tool = True
        self.b_bot.robot_status.held_tool_id = "knife"
        self.publish_robot_status()

        self.planning_scene_interface.allow_collisions("knife", "knife_holder")
        self.b_bot.go_to_named_pose("knife_pick_up_ready", speed=0.2)
        self.planning_scene_interface.disallow_collisions("knife", "knife_holder")

        self.b_bot.go_to_named_pose("home")

    def unequip_knife(self):
        if self.b_bot.robot_status.carrying_object == "knife":
            rospy.logwarn("Robot is not holding knife")
            return

        self.b_bot.go_to_named_pose("knife_pick_up_ready", speed=0.2)

        equip_pose = conversions.to_pose_stamped(
            "knife_pickup_link", [0.03, 0, 0, 0, 0, 0])

        self.planning_scene_interface.allow_collisions("knife", "knife_holder")
        self.b_bot.go_to_pose_goal(equip_pose, speed=.1, move_lin=True)
        self.planning_scene_interface.disallow_collisions("knife", "knife_holder")
        self.confirm_to_proceed("fix pose")

        self.allow_collisions_with_robot_hand("knife", "b_bot")
        self.b_bot.gripper.open()
        self.b_bot.gripper_group.detach_object("knife")
        self.b_bot.robot_status.carrying_tool = False
        self.b_bot.robot_status.held_tool_id = ""
        self.publish_robot_status()

        self.b_bot.go_to_named_pose("knife_pick_up_ready", speed=0.5)

        self.b_bot.go_to_named_pose("home")

    def equip_spatula(self):
        self.spawn_tool("spatula")
        # self.planning_scene_interface.allow_collisions("knife","")
        # self.planning_scene_interface.allow_collisions("bowl","")
        # self.planning_scene_interface.allow_collisions("cutting_board","")
        # self.planning_scene_interface.allow_collisions("plate","")
        # self.planning_scene_interface.allow_collisions("cucumber","")
        self.planning_scene_interface.allow_collisions("spatula", "")

        ps_approach = geometry_msgs.msg.PoseStamped()
        ps_approach.header.frame_id = "spatula_pickup_link"
        ps_approach.pose.position.x = -.04
        ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0, -pi/3, 0))
        # ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -radians(50), 0))

        ps_in_holder = copy.deepcopy(ps_approach)
        ps_in_holder.pose.position.x = .017

        self.b_bot.gripper.open()
        self.b_bot.go_to_named_pose("tool_pick_ready")
        self.b_bot.go_to_pose_goal(ps_approach, speed=.1, move_lin=True)

        self.b_bot.go_to_pose_goal(ps_in_holder, speed=.1, move_lin=True)
        self.allow_collisions_with_robot_hand("spatula", "b_bot")
        self.b_bot.gripper.close()
        self.b_bot.gripper.attach_object("spatula", with_collisions=True)

        self.b_bot.go_to_pose_goal(ps_approach, speed=.1, move_lin=True)
        self.b_bot.go_to_named_pose("tool_pick_ready")

    def unequip_spatula(self):

        ps_approach = geometry_msgs.msg.PoseStamped()
        ps_approach.header.frame_id = "spatula_pickup_link"
        ps_approach.pose.position.x = -.04
        ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0, -pi/6, 0))

        ps_in_holder = copy.deepcopy(ps_approach)
        ps_in_holder.pose.position.x = .017

        self.b_bot.go_to_named_pose("tool_pick_ready")
        self.b_bot.go_to_pose_goal(ps_approach, speed=.1, move_lin=True)

        self.b_bot.go_to_pose_goal(ps_in_holder, speed=.1, move_lin=True)

        self.b_bot.gripper.open()
        self.b_bot.gripper.detach_object("spatula")
        self.b_bot.gripper.forget_attached_item()
        self.planning_scene_interface.remove_attached_object(name="spatula")
        self.planning_scene_interface.remove_world_object(name="spatula")

        self.b_bot.go_to_pose_goal(ps_approach, speed=.1, move_lin=True)
        self.b_bot.go_to_named_pose("tool_pick_ready")

        self.allow_collisions_with_robot_hand("spatula", "b_bot", allow=False)

# Cutting motions with force control

    def check_vegetable_extremity(self):
        pose = conversions.to_pose_stamped(
            "cutting_board", [0.0, 0.12, 0.02, -pi, pi/2, 0])

        self.b_bot.go_to_pose_goal(pose, end_effector_link=self.knife_center_frame, speed=0.2, move_lin=True)
        # Linear push: move in a linear way until you touch something.

        if self.use_real_robot:
            self.b_bot.linear_push(
                force=2.0, direction='-Y', timeout=30, max_translation=0.2)
        else:
            self.b_bot.move_lin_rel(relative_translation=[0, 0.04, 0], relative_to_tcp=True, end_effector_link=self.knife_center_frame, speed=0.05)

    def cut(self, start_pose, timeout=30):
        self.b_bot.go_to_pose_goal(start_pose, speed=0.1, end_effector_link=self.knife_center_frame)

        if not self.use_gazebo_sim and not self.use_real_robot:
            self.b_bot.move_lin_rel(relative_translation=[0, 0, -0.015])
            return

        start_target_pos = self.b_bot.force_controller.end_effector(tip_link=self.knife_center_frame)

        target_trajectory = traj_utils.compute_sinusoidal_trajectory(start_target_pos, dimension=1, period=3, amplitude=0.02, num_of_points=100)
        
        force_position_selection_matrix = [1, 1, 0, 0.6, 1., 1]
        end_effector_link = self.knife_center_frame

        # [force x, force y, force z, torque x, torque y, torque z]
        target_force = [0, 0, -5, -1.0, 0, 0]

        self.b_bot.force_controller.force_control(target_force=target_force, target_positions=target_trajectory,
                                                  force_position_selection_matrix=force_position_selection_matrix,
                                                  end_effector_link=end_effector_link, timeout=timeout)

    def rl_slice(self, slicing_pose, num_of_slices=1, slice_width=0.01, forward_push=0.0, downward_push=0.035):

        self.b_bot.go_to_pose_goal(slicing_pose, speed=0.2, end_effector_link=self.knife_center_frame, move_lin=True)

        self.confirm_to_proceed("Start slicing")

        init_pose = copy.deepcopy(slicing_pose)

        for _ in range(num_of_slices):
            goal_pose = self.b_bot.move_lin_rel(relative_translation=[downward_push, 0, forward_push], 
                                                relative_to_tcp=True, pose_only=True, end_effector_link=self.knife_center_frame)
            goal_pose = self.listener.transformPose("b_bot_base_link", goal_pose)
            print("current pose", self.b_bot.force_controller.end_effector(tip_link="b_bot_knife_center")[:3].tolist())
            print((conversions.from_pose_to_list(goal_pose.pose)[:3]-self.b_bot.force_controller.end_effector(tip_link="b_bot_knife_center")[:3]).tolist())
            self.slicing_rl.execute_policy(target_pose=conversions.from_pose_to_list(goal_pose.pose))

            seq = []
            seq.append(helpers.to_sequence_item_relative(pose=[0, slice_width, 0.01, 0, 0, 0], speed=1.0, retime=True))
            seq.append(helpers.to_sequence_item(pose=init_pose, speed=1.0, retime=True, end_effector_link=self.knife_center_frame))
            seq.append(helpers.to_sequence_item_relative(pose=[0, -slice_width, 0.0, 0, 0, 0], speed=1.0, retime=True))
            self.execute_sequence("b_bot", seq, "reset")

            init_pose.pose.position.y -= slice_width

    def slice_vegetable(self, num_slices=5, slice_width=0.01, timeout=35, slice_type="straight"):
        self.check_vegetable_extremity()

        self.confirm_to_proceed("check pose")

        if slice_type == "straight":
            relative_translation = [-0.02, 0.0, 0.0]
            relative_rotation = [0, 0, 0]
        elif slice_type == "diagonal":
            relative_translation = [-0.02, 0.02, 0.0]
            relative_rotation = [radians(-30), 0, 0]
        else:
            raise Exception("Unknown slice_type: %s" % slice_type)

        self.b_bot.move_lin_rel(relative_translation=relative_translation, relative_rotation=relative_rotation, speed=0.2,
                                relative_to_tcp=True, end_effector_link=self.knife_center_frame)

        after_cut_offset = 0.01
        cut_orientation = self.b_bot.get_current_pose().orientation

        for _ in range(num_slices):
            pose_side = self.b_bot.move_lin_rel(relative_translation=[0.0, -(slice_width+after_cut_offset), 0.0], speed=0.2, pose_only=True)
            pose_side.pose.orientation = cut_orientation

            # cut
            self.cut(pose_side, timeout)

            self.b_bot.move_lin_rel(relative_translation=[0, after_cut_offset, 0.01], relative_rotation=[radians(-15), 0, 0], speed=0.05, retime=True)
            pose_up = self.b_bot.move_lin_rel(relative_translation=[0, 0.0, 0.03], speed=0.5, pose_only=True)
            pose_up.pose.orientation = cut_orientation
            self.b_bot.go_to_pose_goal(pose_up, speed=0.1)

    def cut_tomato_in_half_with_force_control(self, half=False):
        # cut_tomato_with_force_control
        pose_1 = conversions.to_pose_stamped(
            "cutting_board", [-0.010, 0.226, 0.100, -0.490, 0.498, 0.515, 0.497])
        if half:
            pose_2 = conversions.to_pose_stamped(
                "cutting_board", [0.005, 0.145, 0.099, -0.474, 0.506, 0.532, 0.486])
        else:
            pose_2 = conversions.to_pose_stamped(
                "cutting_board", [-0.005, 0.145, 0.099, -0.474, 0.506, 0.532, 0.486])

        self.b_bot.go_to_pose_goal(pose_1, speed=0.2, move_lin=True)
        self.b_bot.go_to_pose_goal(pose_2, speed=0.05, move_lin=True)

        if half:
            number_of_cuts = 1
        else:
            number_of_cuts = 4

        for i in range(number_of_cuts):

            pose_3 = copy.deepcopy(pose_2)
            pose_3.pose.position.x += i * 0.006
            self.b_bot.go_to_pose_goal(pose_3, speed=0.2, move_lin=True)

            self.cut(start_pose=pose_3, timeout=35)

            self.b_bot.move_lin_rel(relative_translation=[
                                    0.0, 0.12, 0.02], speed=0.05)
            self.b_bot.move_lin_rel(relative_translation=[
                                    0.0, 0.0, 0.065], speed=0.2)

        self.b_bot.move_lin_rel(relative_translation=[
                                0.0, 0.10, 0.0], speed=0.2)
        self.b_bot.go_to_named_pose("home")

    def cut_cucumber_in_half(self, slicing_pose):
        self.equip_knife()

        self.b_bot.go_to_pose_goal(slicing_pose, speed=0.2, end_effector_link=self.knife_center_frame, move_lin=True)

        self.cut(slicing_pose, timeout=30)

        self.b_bot.move_lin_rel(relative_translation=[0, 0, 0.05], speed=0.05)

        self.b_bot.go_to_named_pose("home")

# Manipulation of vegetables

    def pick_and_place_in_cutting_board(self, object_id, grasp_pose="grasp_pose", robot_name="a_bot"):
        obj_id, obj_name = self.get_object_info(object_id)

        object_pose = self.look_and_get_object_pose(object_id=obj_id, robot_name=robot_name, frame_id="tray_center", multiple_views=False)

        if not object_pose:
            return False

        self.spawn_object(obj_name, object_pose, object_reference_frame="tray_center")

        self.planning_scene_interface.allow_collisions(obj_name, "")
        grasp_pose = self.get_transformed_grasp_pose(obj_name, grasp_pose, target_frame="workspace_center")
        if not self.simple_pick(robot_name, grasp_pose, item_id_to_attach=obj_name, axis="z", sign=+1, grasp_width=0.085,
                                attach_with_collisions=True, speed_fast=0.25, approach_with_move_lin=True):
            return False

        place_pose = conversions.to_pose_stamped("cutting_board_surface", [0, 0, 0, tau/4, tau/4, 0])

        return self.simple_place(robot_name, place_pose, axis="z", sign=+1, place_height=0.02,
                                 speed_fast=0.5, approach_height=0.1, move_lin=True)

    def hold_vegetable(self, object_name, grasp_pose, robot_name="a_bot"):
        return self.simple_pick(robot_name, grasp_pose, item_id_to_attach=object_name, axis="x",
                                sign=-1, attach_with_collisions=True, approach_height=0.1,
                                lift_up_after_pick=False, speed_fast=0.25,  grasp_height=-0.0,
                                approach_with_move_lin=True)

    def loop_pick_cut_cucumber(self):
        obj_id = 3  # 1 is for cucumber, 2 for tomato
        i = 0
        repetitions = 5
        while i < repetitions:
            obj_pose = self.look_and_get_object_pose(object_id=obj_id, robot_name="a_bot", frame_id="cutting_board", multiple_views=False)
            if obj_pose:
                self.pick_cut_cucumber(object_pose=obj_pose)
            else:
                break
            if i == repetitions-1:
                while True:
                    print("Do you want to loop %d more times? write 'Y' or 'N'" % repetitions)
                    ans = input().lower()
                    if ans == "y":
                        # return self.loop_pick_cut_cucumber()
                        i = 0
                        break
                    elif ans == "n":
                        break
            i = i + 1
        self.a_bot.go_to_named_pose("home")
        return True

    def pick_cut_cucumber(self, object_pose, robot_name="a_bot"):
        grasp_pose = copy.deepcopy(object_pose)
        grasp_pose.pose.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0, tau/4, -tau/4))
        place_pose = conversions.to_pose_stamped(
            "cutting_board", [0.363, -0.081, -0.056, tau/4, tau/4, 0])

        if not self.simple_pick("a_bot", grasp_pose, item_id_to_attach="", axis="z",
                                sign=1, attach_with_collisions=False, approach_height=0.1,
                                lift_up_after_pick=True, speed_fast=0.7, speed_slow=0.5,  grasp_height=-0.018,
                                approach_with_move_lin=True, gripper_force=15, grasp_width=0.06):
            return False

        return self.simple_place("a_bot", place_pose, axis="z", sign=+1, place_height=0.02,
                                 speed_fast=0.6, approach_height=0.1, move_lin=True)

    def move_cucumber_to_tray(self, place_pose=None):
        self.b_bot.go_to_named_pose("home")
        self.a_bot.move_lin_rel(relative_translation=[0.0, 0.0, 0.1])

        if place_pose:
            pose_2 = place_pose
        else:
            pose_2 = conversions.to_pose_stamped(
                "tray_center", [0.0, 0.0, 0.015, 0.737, 0.489, -0.388, 0.261])

        pose_1 = copy.deepcopy(pose_2)
        pose_1.pose.position.z += 0.15

        self.a_bot.go_to_pose_goal(pose_1, speed=0.2, move_lin=True)
        self.a_bot.go_to_pose_goal(pose_2, speed=0.2, move_lin=True)

        self.a_bot.gripper.open(opening_width=0.1)
        return True

    def move_cucumber_to_dish(self):
        self.b_bot.go_to_named_pose("home")
        self.a_bot.move_lin_rel(relative_translation=[0.0, 0.0, 0.1])

        pose_1 = conversions.to_pose_stamped(
            "cutting_board", [0.35, 0.07, 0.144, 0.737, 0.489, -0.388, 0.261])
        pose_2 = conversions.to_pose_stamped(
            "cutting_board", [0.35, 0.07, -0.056, 0.737, 0.489, -0.388, 0.261])

        self.a_bot.go_to_pose_goal(pose_1, speed=0.2, move_lin=True)
        self.a_bot.go_to_pose_goal(pose_2, speed=0.2, move_lin=True)

        self.a_bot.gripper.open(opening_width=0.1)
        self.a_bot.robot_group.detach_object("cucumber")
        self.despawn_object("cucumber")
        self.a_bot.go_to_named_pose("home")
        return True

    def pick_cucumber_b_bot(self, cucumber_tip_pose=None):
        self.allow_collisions_with_robot_hand("cucumber", "b_bot")
        initial_pick_pose = conversions.to_pose_stamped(
            "cutting_board", [-0.019, 0.066, 0.016, 0.507, 0.503, -0.498, 0.493])
        if cucumber_tip_pose:
            initial_pick_pose.pose.position.y = cucumber_tip_pose.pose.position.y + 0.03

        for i in range(6):
            grasp_pose = copy.deepcopy(initial_pick_pose)
            grasp_pose.pose.position.y -= 0.015 * i

            self.simple_pick("b_bot", grasp_pose, axis="z",
                             sign=+1, attach_with_collisions=True, approach_height=0.1,
                             gripper_force=15, lift_up_after_pick=True, speed_fast=0.25,  grasp_height=0.0,
                             approach_with_move_lin=True)

            pose_1 = conversions.to_pose_stamped(
                "cutting_board", [0.363, -0.081, 0.144, -0.339, 1.532, -2.774])
            pose_2 = conversions.to_pose_stamped(
                "cutting_board", [0.33+0.01*i, -0.085, -0.056, -0.339, 1.532, -2.774])

            self.b_bot.go_to_pose_goal(pose_1, speed=0.2, move_lin=True)
            self.b_bot.go_to_pose_goal(pose_2, speed=0.2, move_lin=True)

            self.b_bot.gripper.open(opening_width=0.1)
        self.b_bot.go_to_named_pose("home")

    def move_tomato_to_dish(self):
        self.b_bot.go_to_named_pose("home")
        self.a_bot.move_lin_rel(relative_translation=[0.0, 0.0, 0.1])

        pose_1 = conversions.to_pose_stamped(
            "cutting_board", [0.363, -0.081, 0.144, 0.737, 0.489, -0.388, 0.261])
        pose_2 = conversions.to_pose_stamped(
            "cutting_board", [0.363, -0.081, -0.056, 0.737, 0.489, -0.388, 0.261])

        self.a_bot.go_to_pose_goal(pose_1, speed=0.2, move_lin=True)
        self.a_bot.go_to_pose_goal(pose_2, speed=0.2, move_lin=True)

        self.a_bot.gripper.open(opening_width=0.1)
        self.a_bot.go_to_named_pose("home")
        return True

    def scoop_artichoke(self):
        self.planning_scene_interface.allow_collisions("artichoke", "spatula")
        ps_approach = geometry_msgs.msg.PoseStamped()
        ps_approach.header.frame_id = "cutting_board_surface"
        ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(pi, radians(20), -pi/2))
        ps_approach.pose.position.x -= 0.05
        ps_approach.pose.position.z = .05
        ps_approach.pose.position.y = .09
        ps_below_artichoke = copy.deepcopy(ps_approach)
        ps_below_artichoke.pose.position.y = 0.09
        ps_below_artichoke.pose.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(pi, radians(00), -pi/2))
        ps_below_artichoke.pose.position.z = .05

        ps_scoop_artichoke = copy.deepcopy(ps_below_artichoke)
        ps_scoop_artichoke.pose.position.z -= .025
        ps_scoop_artichoke.pose.position.y = .11

        ps_scoop2_artichoke = copy.deepcopy(ps_scoop_artichoke)

        ps_scoop2_artichoke.pose.position.y = 0

        self.b_bot.go_to_pose_goal(
            ps_approach, speed=.1, move_lin=True, end_effector_link="b_bot_spatula_center")

        self.b_bot.go_to_pose_goal(
            ps_below_artichoke, speed=.1, move_lin=True, end_effector_link="b_bot_spatula_center")

        self.b_bot.go_to_pose_goal(
            ps_scoop_artichoke, speed=.1, move_lin=True, end_effector_link="b_bot_spatula_center")

        self.b_bot.go_to_pose_goal(
            ps_scoop2_artichoke, speed=.1, move_lin=True, end_effector_link="b_bot_spatula_center")

        self.b_bot.move_lin_rel(relative_translation=[0.0, 0.0, 0.075])

    def look_then_pick_and_place(self, robot_name, object_id, grasp_pose="grasp_pose", source_loc="tray_center", destination_loc="cutting_board_surface", place_pose=[0, 0, 0.02, tau/4, tau/4, 0]):
        obj_id, obj_name = self.get_object_info(object_id)

        # Locate object
        object_pose = self.look_and_get_object_pose(object_id=obj_id, robot_name=robot_name, frame_id="tray_center", multiple_views=False)
        print(object_pose)
        if not object_pose:
            return False

        # Visualize in Rviz and Planning Scene
        self.spawn_object(obj_name, object_pose, object_reference_frame="tray_center")

        grasp_ps = self.get_transformed_grasp_pose(obj_name, grasp_pose, target_frame="workspace_center")
        return self.pick_and_place_object(robot_name, obj_name, grasp_ps, destination_loc, place_pose=place_pose)

    def pick_and_place_object(self, robot_name, object_name, grasp_pose, destination_loc="cutting_board_surface", place_pose=[0, 0, 0.02, tau/4, tau/4, 0]):
        # Pick object
        self.planning_scene_interface.allow_collisions(object_name, "")

        # Get grasping pose from object in the planning scene
        if not isinstance(grasp_pose, geometry_msgs.msg.PoseStamped):
            raise Exception("Invalid grasp pose")

        if not self.simple_pick(robot_name, grasp_pose, item_id_to_attach=object_name, axis="z", sign=+1, grasp_width=0.085,
                                attach_with_collisions=True, speed_fast=0.25, approach_with_move_lin=True):
            return False

        # Place object
        place_pose = conversions.to_pose_stamped(destination_loc, place_pose)

        self.simple_place(robot_name, place_pose, axis="z", sign=+1, place_height=0.0,
                          speed_fast=0.5, approach_height=0.1, move_lin=False)

        self.planning_scene_interface.disallow_collisions(object_name, "")

        return True

    def cut_cherry_tomato(self):
        self.planning_scene_interface.allow_collisions("knife", "cherry_tomato")
        self.allow_collisions_with_robot_hand("knife", "a_bot")

        self.equip_knife()

        pose_1 = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.15, 0.03, pi/2, pi/2, 0])
        self.b_bot.go_to_pose_goal(pose_1, end_effector_link=self.knife_center_frame, move_lin=True, speed=0.2)

        slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.06, 0.03, pi/2, pi/2, 0])
        self.b_bot.go_to_pose_goal(slicing_pose, speed=0.2, end_effector_link=self.knife_center_frame, move_lin=True)

        self.cut(slicing_pose, timeout=30)

        self.b_bot.move_lin_rel(relative_translation=[0, 0.05, 0.05], speed=0.05)
        self.allow_collisions_with_robot_hand("knife", "a_bot", allow=False)

######## DEMOS ########

    def cucumber_sticks_demo(self):
        self.use_dummy_vision = True

        # pick and place cucumber in cutting board (OK)
        self.look_then_pick_and_place(robot_name="a_bot", object_id="cucumber")

        # Hold cucumber
        grasp_pose = conversions.to_pose_stamped("move_group/cucumber/tip1", [0.01, 0.0, 0.02, 0, 0.299, 0, 0.954])
        if not self.hold_vegetable("cucumber", grasp_pose):
            return False

        # cut cucumber in half
        # in half perpendicular
        self.planning_scene_interface.allow_collisions("knife", "cucumber")
        slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.01, 0.05, -pi, pi/2, 0])
        self.cut_cucumber_in_half(slicing_pose)

        # pick one half (the one a_bot is already holding) and place it back in the table
        self.a_bot.gripper.detach_object("cucumber")
        self.despawn_object("cucumber")

        self.spawn_object("half_cucumber", object_pose=[0, 0, 0.015, tau/4, tau/4, 0], object_reference_frame="cutting_board_surface", alias="half1")
        self.spawn_object("half_cucumber", object_pose=[-0.009, 0.001, 0.015, -tau/4, tau/4, 0], object_reference_frame="cutting_board_surface", alias="half2")

        rospy.sleep(0.1)
        self.a_bot.gripper.attach_object(object_to_attach="half1", with_collisions=True)
        self.a_bot.gripper.forget_attached_item()

        self.planning_scene_interface.allow_collisions("half1", "half2")

        self.confirm_to_proceed("go")

        place_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.015, 0.737, 0.489, -0.388, 0.261])
        self.move_cucumber_to_tray(place_pose=place_pose)
        self.a_bot.gripper.detach_object(object_to_detach="half1")
        self.planning_scene_interface.remove_attached_object(name="half1")
        self.a_bot.move_lin_rel([0, 0, 0.15])

        # for-loop pick / hold / cut / put in dish
        cucumber_parts = {
            "half2": "tip2",
            "half1": "tip2",
        }
        for part_name, grasp_point in cucumber_parts.items():
            print(part_name, grasp_point)
            # pick one part and hold it
            grasp_pose = self.get_transformed_grasp_pose("half_cucumber", "grasp_center", target_frame="workspace_center", alias=part_name)
            grasp_pose.pose.orientation = helpers.rotateQuaternionByRPY(tau/2, 0, 0, grasp_pose.pose.orientation)
            self.pick_and_place_object(robot_name="a_bot", object_name=part_name, grasp_pose=grasp_pose, destination_loc="cutting_board_surface",
                                       place_pose=[0, -0.04, 0.015, tau/4, tau/4, 0])
            # Hold half cucumber
            grasp_pose = conversions.to_pose_stamped("move_group/%s/%s" % (part_name, grasp_point), [-0.01, 0.0, -0.009, 0, 0.299, 0, 0.954])
            if not self.hold_vegetable(part_name, grasp_pose):
                return False

            self.confirm_to_proceed("next")

            # cut cucumber in half with an inclination
            # in half parallel
            self.allow_collisions_with_robot_hand("knife", "a_bot")
            self.planning_scene_interface.allow_collisions("knife", part_name)
            slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.02, 0.04, pi/2, pi/2, 0])
            self.cut_cucumber_in_half(slicing_pose)
            self.allow_collisions_with_robot_hand("knife", "a_bot", allow=False)

            # pick and place sticks on the dish
            place_pose = conversions.to_pose_stamped("tray_center", [0.05, -0.05, 0.02, 0.737, 0.489, -0.388, 0.261])
            self.move_cucumber_to_tray(place_pose)
            self.a_bot.gripper_group.detach_object(part_name)
            self.a_bot.gripper.forget_attached_item()
            self.planning_scene_interface.remove_world_object(name=part_name)
            self.a_bot.move_lin_rel([0, 0, 0.15])

        self.a_bot.go_to_named_pose("home")

        self.unequip_knife()
        self.b_bot.go_to_named_pose("home")

    def cucumber_demo(self):
        if not self.pick_and_place_in_cutting_board("cucumber", "grasp_1", "a_bot"):
            return False

        grasp_pose = conversions.to_pose_stamped("move_group/cucumber/tip1", [0.01, -0.02, -0.009, 0, 0.299, 0, 0.954])

        if not self.hold_vegetable("cucumber", grasp_pose):
            return False

        self.confirm_to_proceed("cutting cucumber")
        self.equip_knife()
        self.slice_vegetable(num_slices=10, slice_width=0.01, timeout=35)
        self.b_bot.go_to_named_pose("home")
        if not self.move_cucumber_to_dish():
            return False
        if not self.loop_pick_cut_cucumber():
            return False
        self.ab_bot.go_to_named_pose("home")

    def cherry_tomato_demo(self):
        self.use_dummy_vision = True

        self.allow_collisions_with_robot_hand("base_fixture_top", "a_bot")

        # pick and place cucumber in cutting board (OK)
        self.dummy_pose = [0.0, 0.0, 0.0] + np.deg2rad([0, 90., 0]).tolist()
        self.look_then_pick_and_place(robot_name="a_bot", object_id="cherry_tomato", place_pose=[0, 0, 0.005, tau/4, tau/4, 0])

        grasp_pose = conversions.to_pose_stamped("move_group/cherry_tomato", [-0.005, 0.0, -0.01, -0.000, 0.288, 0.000, 0.958])
        if not self.hold_vegetable("cherry_tomato", grasp_pose):
            return False

        # cut in half
        self.cut_cherry_tomato()
        self.b_bot.go_to_named_pose("home")

        self.a_bot.gripper.open()
        self.a_bot.gripper_group.detach_object("cherry_tomato")
        self.a_bot.gripper.forget_attached_item()

        grasp_pose = self.get_transformed_grasp_pose("cherry_tomato", "grasp_1", target_frame="workspace_center")
        self.pick_and_place_object(robot_name="a_bot", object_name="cherry_tomato", grasp_pose=grasp_pose, destination_loc="cutting_board_surface",
                                       place_pose=[0, 0.0, 0.005, tau/2, tau/4, 0])

        # hold again
        grasp_pose = conversions.to_pose_stamped("move_group/cherry_tomato", [-0.005, 0.0, -0.01, -0.000, 0.288, 0.000, 0.958])
        grasp_pose.pose.orientation = helpers.rotateQuaternionByRPYInUnrotatedFrame(-tau/4, 0, 0, grasp_pose.pose.orientation)
        if not self.hold_vegetable("cherry_tomato", grasp_pose):
            return False

        # cut in half
        self.cut_cherry_tomato()

        self.unequip_knife()
        self.b_bot.go_to_named_pose("home")

        place_pose = conversions.to_pose_stamped("tray_center", [0.05, -0.05, 0.015, 0.737, 0.489, -0.388, 0.261])
        self.move_cucumber_to_tray(place_pose)
        self.a_bot.gripper_group.detach_object("cherry_tomato")
        self.planning_scene_interface.remove_world_object(name="cherry_tomato")
        self.a_bot.move_lin_rel([0, 0, 0.15])
        self.a_bot.go_to_named_pose("home")

    def tomato_demo(self):
        if not self.pick_and_place_in_cutting_board("tomato", "grasp_1", "a_bot"):
            return False
        grasp_pose = conversions.to_pose_stamped("cutting_board", [0.0, 0.01, 0.028, 0.623, 0.623, -0.334, 0.335])
        if not self.hold_vegetable("tomato", grasp_pose):
            return False
        self.confirm_to_proceed("cutting tomato")
        self.equip_knife()
        self.cut_tomato_in_half_with_force_control()
        self.b_bot.go_to_named_pose("home")
        self.move_tomato_to_dish()
        self.ab_bot.go_to_named_pose("home")

    def demo_2(self):
        # Tomato: Pick -> hold -> Pick again -> cut -> rotate -> hold -> cut -> move to dish
        # self.pick_tomato()
        # self.hold_tomato()

        # self.confirm_to_proceed("cutting tomato")

        # self.cut_tomato_in_half_with_force_control(half=True)
        # self.b_bot.go_to_named_pose("home")

        # self.confirm_to_proceed("rotate tomato")

        # self.pick_tomato(rotate=True)
        self.hold_tomato()

        self.confirm_to_proceed("cutting tomato")

        self.cut_tomato_in_half_with_force_control(half=True)

        self.confirm_to_proceed("move tomato")

        self.move_tomato_to_dish()

        # # reset
        self.reset_scene_and_robots()  # Only remove objects
        self.ab_bot.go_to_named_pose("home")  # Move robots to home pose
        self.load_objects()  # Load the objects again

        self.confirm_to_proceed("Start cucumber")

        # Cucumber: Pick -> hold -> cut in half -> cut diagonal -> pick -> move to dish
        self.pick_cucumber()
        self.hold_cucumber()

        self.confirm_to_proceed("cutting cucumber")

        self.cut_cucumber_in_half_with_force_control(half=True)

        self.confirm_to_proceed("Cut cucumber")

        cucumber_tip_pose = self.cut_cucumber_diag_with_force_control()

        self.confirm_to_proceed("unequip knife")

        self.unequip_knife_2()

        self.confirm_to_proceed("move cucumber")

        self.pick_cucumber_b_bot(cucumber_tip_pose)

        self.move_cucumber_to_dish()

    def collect_dataset_tray(self, robot_name="a_bot"):
        # offsets x_offset = .055  y_offset = .095
        height = 0.4
        self.activate_led(robot_name)
        tray_view_high, close_tray_views = self.define_local_tray_views(robot_name=robot_name, include_rotated_views=True,
                                                                        high_height=0.385, low_height=height,
                                                                        offsets=[0.045, 0.085])
        tray_views = [tray_view_high] + close_tray_views

        for view in tray_views:
            assert not rospy.is_shutdown()
            self.vision.activate_camera(robot_name + "_outside_camera")
            self.active_robots[robot_name].go_to_pose_goal(
                view, end_effector_link=robot_name + "_outside_camera_color_frame", speed=.5, acceleration=.3, wait=True, move_lin=True)
            rospy.sleep(0.5)
            self.write_cam_image("tomato_%s" % height)
        return False

    def collect_dataset_cutting_board(self, robot_name="a_bot"):
        height = 0.4
        self.activate_led(robot_name)
        tray_view_high, close_tray_views = self.define_local_tray_views(robot_name=robot_name, include_rotated_views=True,
                                                                        high_height=0.385, low_height=height,
                                                                        frame_id="cutting_board",
                                                                        offsets=[0.04, 0.05])
        tray_views = [tray_view_high] + close_tray_views

        for view in tray_views:
            assert not rospy.is_shutdown()
            self.vision.activate_camera(robot_name + "_outside_camera")
            self.active_robots[robot_name].go_to_pose_goal(
                view, end_effector_link=robot_name + "_outside_camera_color_frame", speed=.5, acceleration=.3, wait=True, move_lin=True)
            rospy.sleep(0.5)
            self.write_cam_image("cut_cucumber_%s" % height)

        return False
