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

import signal
from o2ac_routines.cooking import Cooking
import time
import sys
import rospy

from ur_control import conversions

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    try:
        rospy.init_node('o2ac_routines', anonymous=False)
        rospy.loginfo(" ...")
        c = Cooking()
        while True:
            rospy.loginfo("Enter 1 for both robots to go home.")
            rospy.loginfo("Enter 1a for a bot to go home.")
            rospy.loginfo("Enter 1b for b bot to go home.")
            rospy.loginfo("Enter 2 to launch cucumber demo")
            rospy.loginfo("Enter 3 to launch tomato demo")
            rospy.loginfo("Enter 4 to launch cucumber/tomato demo")
            rospy.loginfo("Enter 5a to pick and place cucumber from tray to cooking board")
            rospy.loginfo("Enter 5b to pick and place tomato from tray to cooking board")
            rospy.loginfo("Enter 6a to pick and place cut cucumber in plate")
            rospy.loginfo("Enter 6b to pick and place cut tomato in plate")
            rospy.loginfo("Enter 7a to view cooking board")
            rospy.loginfo("Enter 7b to view tray")
            rospy.loginfo("Enter 8 to check vegetable extremity with linear push")
            rospy.loginfo("Enter 10 go to hold cucumber pose")
            rospy.loginfo("Enter 'equip' to equip knife")
            rospy.loginfo("Enter 'unequip' to unequip knife")
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

            elif i == '2':  # cucumber demo
                c.cucumber_demo()

            elif i == '3':  # tomato demo
                c.tomato_demo()

            elif i == '4':  # cucumber/tomato demo
                c.cucumber_demo()
                c.tomato_demo()
                # TODO: change to parallel implementation, not series

            elif i == '5a':  # pick and place cucumber from tray to cooking board
                # c.use_dummy_vision = True
                c.pick_and_place_in_cutting_board("cucumber", "grasp_1", "a_bot")

            elif i == '5b':  # pick and place tomato from tray to cooking board
                c.pick_and_place_in_cutting_board("tomato", "grasp_1", "a_bot")

            elif i == '6a':  # pick and place cut cucumber in plate
                c.loop_pick_cut_cucumber()

            elif i == '6b':  # pick and place cut tomato in plate
                c.move_tomato_to_dish()  # simple pick and place to plate
                # TODO: vision-based pick & place

            elif i == '7a':  # view cooking board
                c.look_and_get_object_pose(object_id=0, robot_name="a_bot", frame_id="cutting_board", multiple_views=False)

            elif i == '7ab':  # view cooking board
                c.look_and_get_object_pose(object_id=0, robot_name="b_bot", frame_id="cutting_board", multiple_views=False)

            elif i == '7b':  # view tray
                c.look_and_get_object_pose(object_id=0, robot_name="a_bot", frame_id="tray_center", multiple_views=False)

            elif i == '8':  # check extremity with linear push
                c.check_vegetable_extremity()

            elif i == '9a': # cucumber
                c.visualize_object(1)

            elif i == '10a': # hold cucumber
                grasp_pose = conversions.to_pose_stamped("move_group/cucumber/tip1", [0.01, -0.02, -0.009, 0, 0.299, 0, 0.954])
                c.hold_vegetable("cucumber", grasp_pose)

            elif i == 'equip':  # equip knife
                c.equip_knife()

            elif i == 'unequip':  # unequip knife
                c.unequip_knife()

            elif i == 'reset':
                c.reset_scene_and_robots()
            elif i == 'x':
                break
            elif i == "":
                continue
            print("This took: %.3f seconds" % (time.time() - tic_start))
    except rospy.ROSInterruptException:
        pass
