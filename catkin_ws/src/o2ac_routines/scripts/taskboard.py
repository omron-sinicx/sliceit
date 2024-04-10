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
# Author: Felix von Drigalski, Cristian C. Beltran-Hernandez


import signal
import sys
from o2ac_routines.taskboard import O2ACTaskboard
import time
import rospy
import geometry_msgs.msg
import tf_conversions
from math import pi

tau = 2.0*pi  # Part of math from Python 3.6


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    try:
        rospy.init_node('o2ac_routines', anonymous=False)
        c = O2ACTaskboard()
        c.define_tool_collision_objects()
        i = 1
        while i and not rospy.is_shutdown():
            rospy.loginfo("Enter 1 to move robots to home")
            rospy.loginfo("Enter 11, 12 to close/open grippers")
            rospy.loginfo("Enter 13, 14 to equip/unequip m3 screw tool (133,144: m4)")
            rospy.loginfo("Enter 15, 16 to equip/unequip m4 screw tool (151,161: padless)")
            rospy.loginfo("Enter 21, 22, 23: pick m3, m4 screw, (a_bot M4)")
            rospy.loginfo("Enter 3 to calibrate TB screw tasks (31: M3, 32: M4)")
            rospy.loginfo("Subtasks: 51 (set screw), 52 (M3), 53 (M4), 54 (belt), 55 (motor pulley), 56 (shaft), 57 (bearing), 58 (idler pulley)")
            rospy.loginfo("Enter 8 to spawn example parts")
            rospy.loginfo("Enter reset to reset the scene")
            rospy.loginfo("Enter prep to prepare the task (do this before running)")
            rospy.loginfo("Enter ssup/ssdown to fine-tune the set screw tool position")
            rospy.loginfo("Enter start to run the task (competition mode, no confirmations)")
            rospy.loginfo("Enter test for a test run of the task (WITH confirmations)")
            rospy.loginfo("Enter x to exit")
            i = input()
            tic_start = time.time()
            if i == "prep":
                c.prep_taskboard_task_simultaneous()
            if i == "simpleprep":
                c.prep_taskboard_task()
            if i == "ssprep":
                c.move_b_bot_to_setscrew_initial_pos()
            if i == "ssup":
                c.at_set_screw_hole.pose.position.z -= 0.001
                c.move_b_bot_to_setscrew_initial_pos()
            if i == "ssdown":
                c.at_set_screw_hole.pose.position.z += 0.001
                c.move_b_bot_to_setscrew_initial_pos()
            #########################
            # Simultaneous tasks
            #########################
            if i == "start":
                c.competition_mode = True
                c.full_taskboard_task_simultaneous(do_screws=True, skip_tray_placing=False)
                c.competition_mode = False
            if i == "startnocarry":
                c.competition_mode = True
                c.full_taskboard_task_simultaneous(do_screws=True, skip_tray_placing=True)
                c.competition_mode = False
            if i == "screwsonly" or i == "screws":
                c.competition_mode = True
                c.do_screw_tasks_simultaneous()
                c.competition_mode = False
            if i == "startnoscrews":
                c.competition_mode = True
                c.full_taskboard_task_simultaneous(do_screws=False, skip_tray_placing=True)
                c.competition_mode = False
            #########################
            # Non-simultaneous versions
            #########################
            if i == "simplestart":
                c.competition_mode = True
                c.full_taskboard_task(skip_tray_placing=False)
                c.competition_mode = False
            if i == "simplenoscrews":
                c.competition_mode = True
                c.full_taskboard_task(do_screws=False)
                c.competition_mode = False
            if i == "simplescrewsonly":
                c.competition_mode = True
                c.do_screw_tasks_from_prep_position()
                c.competition_mode = False
            #########################
            # Miscellaneous useful commands
            #########################
            if i == "test":
                c.competition_mode = False
                c.full_taskboard_task(skip_tray_placing=False)
            if i == "testnoscrews":
                c.competition_mode = False
                c.full_taskboard_task(do_screws=False)
            if i == "carry":
                c.center_tray_stack(spawn_single_tray=True)
                c.pick_tray_from_agv_stack_calibration_long_side(tray_name="tray2")
            if i == "1":
                c.ab_bot.go_to_named_pose("home")
            if i == "11":
                c.a_bot.gripper.close(wait=False)
                c.b_bot.gripper.close(wait=False)
            if i == "12":
                c.a_bot.gripper.open(wait=False)
                c.b_bot.gripper.open(wait=False)
            #########################
            # Tools
            #########################
            if i == "13":
                c.equip_tool("a_bot", "screw_tool_m3")
            if i == "14":
                c.unequip_tool("a_bot", "screw_tool_m3")
            if i == "133":
                c.equip_tool("a_bot", "screw_tool_m4")
            if i == "144":
                c.unequip_tool("a_bot", "screw_tool_m4")
            if i == "15":
                c.equip_tool("b_bot", "screw_tool_m4")
            if i == "16":
                c.unequip_tool("b_bot", "screw_tool_m4")
            if i == "151":
                c.equip_tool("b_bot", "padless_tool_m4")
            if i == "161":
                c.unequip_tool("b_bot", "padless_tool_m4")
            if i == "17":
                c.equip_tool("b_bot", "set_screw_tool")
            if i == "18":
                c.unequip_tool("b_bot", "set_screw_tool")
            if i == "3":
                c.b_bot.go_to_named_pose("home")
                c.do_task("M3 screw", fake_execution_for_calibration=True)
                c.a_bot.go_to_named_pose("home")
                c.do_task("M4 screw", fake_execution_for_calibration=True)
                c.b_bot.go_to_named_pose("home")
            if i == "21":
                c.pick_screw_from_feeder("a_bot", screw_size=3)
            if i == "22":
                c.pick_screw_from_feeder("b_bot", screw_size=4)
            if i == "23":
                c.pick_screw_from_feeder("a_bot", screw_size=4)
            #########################
            # Taskboard individual subtasks
            #########################
            if i == "31":
                c.do_task("M3 screw", fake_execution_for_calibration=True)
            if i == "32":
                c.do_task("M4 screw", fake_execution_for_calibration=True)
            if i == "51":
                c.do_task("M2 set screw")
            if i == "52":
                c.do_task("M3 screw")
            if i == "53":
                c.do_task("M4 screw")
            if i == "54":
                c.do_task("belt")
            if i == "55":
                c.do_task("motor pulley")
            if i == "56":
                c.do_task("shaft")
            if i == "57":
                c.do_task("bearing")
            if i == "577":
                c.do_task("screw_bearing")
            if i == "577a":
                c.fasten_bearing(task="taskboard", robot_name="a_bot", simultaneous=True)
            if i == "579":
                c.fasten_bearing("taskboard", only_retighten=True)
            if i == "575":
                c.align_bearing_holes(task="taskboard")
            if i == "575a":
                c.align_bearing_holes(task="taskboard", robot_name="a_bot")
            if i == "58":
                c.do_task("idler pulley")
            if i == "58s":
                c.do_task("idler pulley", simultaneous=True)
            #########################
            # Miscellaneous
            #########################
            if i == "8":
                c.spawn_example_objects()
            if i == "9":
                c.activate_led("b_bot", on=True)
            if i == "99":
                c.activate_led("b_bot", on=False)
            if i == "d1":
                c.b_bot.check_for_dead_controller_and_force_start()
            if i == "reset":
                c.reset_scene_and_robots()
                c.reset_assembly_visualization()
            if i == "competitionmodeon":
                c.competition_mode = True
            if i == "x":
                break
            print("This took: %.3f seconds" % (time.time() - tic_start))
            i = True

        print("============ Done!=============")
    except rospy.ROSInterruptException:
        pass
