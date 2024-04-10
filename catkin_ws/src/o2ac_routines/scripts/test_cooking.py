#!/usr/bin/env python

from o2ac_routines.cooking import Cooking
import rospy
from ur_control import conversions
import numpy as np
import sys
import signal
from math import pi, radians

from ur_gazebo.gazebo_spawner import GazeboModels
from ur_gazebo.model import Model
from ur_gazebo.basic_models import SPHERE, PEG_BOARD, BOX, SPHERE_COLLISION

tau = 2.0*pi  # Part of math from Python 3.6


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def place_cucumber():
    name = "cucumber"
    object_pose = conversions.to_pose_stamped("cutting_board_surface", [0, 0.02, 0.0, 0, 0, 0])
    object_pose = c.listener.transformPose("world", object_pose)
    op = conversions.from_pose_to_list(object_pose.pose)
    models = [Model(name, op, reference_frame="world")]
    c.gazebo_scene.load_models(models,)


def place_cube():
    cube_lenght = "0.01"
    obj = BOX % ("box", 0.2, 0.05, cube_lenght,
                 "Yellow", 0.2, 0.05, cube_lenght)
    model_names = "box"
    objpose = [0.1,  0.0,  0.899557, 0, 0.0, 0, 0.0]

    models = [Model(model_names, objpose, file_type='string',
                    string_model=obj, reference_frame="world")]
    c.gazebo_scene.load_models(models)


def test_gazebo():
    name = "panel_bearing"
    object_pose = conversions.to_pose_stamped(
        "tray_center", [0, 0, 0, 0, 0, 0])
    object_pose = c.listener.transformPose("world", object_pose)
    op = conversions.from_pose_to_list(object_pose.pose)
    objpose = [op[:3], op[3:]]
    models = [Model(name, objpose[0], orientation=objpose[1],
                    reference_frame="world")]
    c.gazebo_scene.load_models(models,)
    c.b_bot.gripper.gripper.grab(link_name="panel_bearing_tmp::panel_bearing")
    c.b_bot.gripper.gripper.release(
        link_name="panel_bearing_tmp::panel_bearing")

# No warnings printed
# cmd 2> /dev/null


def main():
    rospy.init_node("testscript")
    global c
    c = Cooking()
    # c.spawn_tool("spatula")
    c.reset_scene_and_robots()  # Only remove objects
    # c.ab_bot.go_to_named_pose("home")

    # test slicing
    slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.01, 0.05, -pi, pi/2, 0])
    c.b_bot.go_to_pose_goal(slicing_pose, speed=0.2, end_effector_link=c.knife_center_frame, move_lin=True)
    # c.confirm_to_proceed("go")
    # c.cut(slicing_pose, timeout=10)
    # c.confirm_to_proceed("go up")
    # c.b_bot.move_lin_rel(relative_translation=[0, 0, -0.04], max_cartesian_speed=0.0001)
    # print("start_pose", move_group.get_current_pose().pose)
    target_pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.01, 0.05, -pi, pi/2, 0])
    c.b_bot.robot_group.limit_max_cartesian_link_speed(speed=0.001, link_name="b_bot_gripper_tip_link")
    trajectory = c.b_bot.robot_group.compute_cartesian_path([target_pose.pose], eef_step=0.001, jump_threshold=1.0)
    c.b_bot.robot_group.clear_max_cartesian_link_speed()
    c.b_bot.robot_group.execute(trajectory)

    #### Check the position of the cucumber's tip
    pose = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.07, 0.005, -pi, pi/2, 0])
    c.b_bot.go_to_pose_goal(pose, end_effector_link=c.knife_center_frame, speed=0.2, move_lin=True)
    c.confirm_to_proceed("linear push")
    c.b_bot.linear_push(force=3.0, direction='+Y', timeout=30, max_translation=0.2)

    # c.cucumber_sticks_demo()

    # RL slice
    slice_width = 0.003
    # Wagiri style
    # slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.035, cucumber_tip.y - slice_width, 0.0345, 0.710, 0.012, -0.704, 0.012])
    # slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.035, 0.06, 0.034, 0.710, 0.012, -0.704, 0.012])
    # Nanamegiri style
    # slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.03, 0.06, 0.035, 0.666, 0.253, -0.656, 0.249])
    slicing_pose = conversions.to_pose_stamped("cutting_board_surface", [0.025, cucumber_tip.y - (slice_width*2), 0.0345, 0.685, 0.196, -0.675, 0.195])
    seq = []
    seq.append(helpers.to_sequence_item_relative([0, 0.005, 0.04, 0, 0, 0], retime=True))
    seq.append(helpers.to_sequence_item(slicing_pose, retime=True, end_effector_link=c.knife_center_frame))
    c.execute_sequence("b_bot", seq, "x")
    c.rl_slice(slicing_pose, num_of_slices=5, slice_width=slice_width, downward_push=0.040)

    # pose_1 = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.15, 0.03, pi/2, pi/2, 0])
    # c.b_bot.go_to_pose_goal(pose_1, end_effector_link=c.knife_center_frame)

    # pose_1 = conversions.to_pose_stamped("cutting_board_surface", [0.0, 0.06, 0.03, pi/2, pi/2, 0])
    # c.b_bot.go_to_pose_goal(pose_1, end_effector_link=c.knife_center_frame)
    # c.confirm_to_proceed("go")

    # c.spawn_tool("knife")

    # c.equip_knife()
    # c.unequip_knife()

    # c.spawn_object("half_cucumber", object_pose=[0, 0, 0.015, tau/4, tau/4, 0], object_reference_frame="cutting_board_surface")
    # c.spawn_object("half_cucumber", object_pose=[-0.009, 0.001, 0.015, -tau/4, tau/4, 0], object_reference_frame="cutting_board_surface", alias="half2")

    # c.pick_and_place_object(robot_name="a_bot", obj_name="half_cucumber", grasp_pose="grasp_center", destination_loc="tray_center", alias="half2",
    #                         place_orientation=[0, tau/4, 0])

    # place_cucumber()
    # c.slice_vegetable(num_slices=1, slice_type="straight")

    # c.ab_bot.go_to_named_pose("home")
    # c.confirm_to_proceed("go")
    # place_cucumber()
    # c.slice_vegetable(num_slices=3, slice_type="diagonal")

    # Motion with different views of the tray.
    # c.collect_db()
    # For cooking board

    # c.reset_scene_and_robots()
    # c.use_dummy_vision = True
    # c.look_then_pick_and_place(robot_name="a_bot", object_id="cucumber")

    # c.cut_cucumber_in_half_with_force_control()

    # c.spawn_object("cucumber", [0,0,0.01,1.5707,0,0], object_reference_frame="cutting_board_surface")
    # c.confirm_to_proceed("next")
    # c.spawn_object("cucumber", [0,0,0.01,1.5707,0,0], object_reference_frame="cutting_board_surface", scale=(0.002,0.002,0.001))

    # pose = conversions.to_pose_stamped("tray_center", [0,0,0,0,0,0])
    # c.spawn_object("cucumber", pose, "tray_center", scale=(0.001,0.001,0.002))
    # c.ab_bot.go_to_named_pose("home")  # Move robots to home pose
    # c.look_and_get_object_pose(object_id=1, robot_name="a_bot", frame_id="tray_center", multiple_views=False)
    # dims = c.object_in_tray_dimensions.get(1)  # width, length

    # if not dims:
    #     return False

    # width, length = dims
    # print("width = ", width)
    # print("length = ", length)
    # # TODO scale mesh using length
    # c.visionary_load_object(c.objects_in_tray)


if __name__ == "__main__":
    main()
