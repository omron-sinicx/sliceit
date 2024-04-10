#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2022, OMRON SINIC X Corp.
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
#  * Neither the name of OMRON SINIC X Corp. nor the names of its
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

# import o2ac_vision.o2ac_ssd
from o2ac_vision.cam_utils import CAMERA_FAILURE, O2ACCameraHelper

import std_msgs.msg
import visualization_msgs.msg
import geometry_msgs.msg
import o2ac_msgs.msg
import sensor_msgs.msg
import cv_bridge  # This offers conversion methods between OpenCV
import cv2
import rospy
import tf
import tf_conversions
import message_filters
import copy
import os
from datetime import datetime
import rospkg
import actionlib

from math import pi

from o2ac_vision.yolov5_cooking import YoloV5Detection

tau = 2.0 * pi  # Part of math from Python 3.6
# and ROS formats
# See here:
#   http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# Note that a similar package exists for PCL:
#   http://wiki.ros.org/pcl_ros


class O2ACVisionServer(object):
    """
    Advertises the vision actions that we will call during the tasks:
    - 2D Pose estimation
    """

    def __init__(self):
        rospy.init_node("o2ac_vision_server", anonymous=False)

        weights_filename = rospy.get_param("~weights_filename")

        # load model YOLOv5
        self.yolov5_model = YoloV5Detection(weights=weights_filename,
                                            imgsz=(640, 640), device='0') # Assuming that there is a GPU

        rospy.loginfo("YOLOv5 pre-trained model loaded")

        self.rospack = rospkg.RosPack()

        # Determine whether the detection server works in continuous mode or
        # not.
        self.continuous_streaming = rospy.get_param("~continuous_streaming", False)

        if self.continuous_streaming:
            rospy.logwarn(
                "Localization action server is not running because SSD results are being streamed! Turn off continuous mode to use localization."
            )
        else:

            # Action server for 2D localization by SSD
            self.get_2d_poses_from_yolov5_server = actionlib.SimpleActionServer(
                "~get_2d_poses_from_ssd",
                o2ac_msgs.msg.get2DPosesFromSSDAction,
                auto_start=False,
            )
            self.get_2d_poses_from_yolov5_server.register_goal_callback(
                self.get_2d_poses_from_yolov5_goal_callback
            )
            self.get_2d_poses_from_yolov5_server.start()

            # Action server for 3D localization by SSD
            self.get_3d_poses_from_yolov5_server = actionlib.SimpleActionServer(
                "~get_3d_poses_from_ssd",
                o2ac_msgs.msg.get3DPosesFromSSDAction,
                auto_start=False,
            )
            self.get_3d_poses_from_yolov5_server.register_goal_callback(
                self.get_3d_poses_from_yolov5_goal_callback
            )
            self.get_3d_poses_from_yolov5_server.start()

        # Setup subscribers of camera topics and synchronizer.
        self.camera_info_sub = message_filters.Subscriber("/camera_info", sensor_msgs.msg.CameraInfo)
        self.image_sub = message_filters.Subscriber("/image", sensor_msgs.msg.Image)
        self.depth_sub = message_filters.Subscriber("/depth", sensor_msgs.msg.Image)
        sync = message_filters.ApproximateTimeSynchronizer([self.camera_info_sub, self.image_sub, self.depth_sub], 10, 0.01)
        sync.registerCallback(self.synced_images_callback)

        self.bridge = cv_bridge.CvBridge()

        # Setup publisher for object detection results
        self.results_pub = rospy.Publisher(
            "~detection_results", o2ac_msgs.msg.Estimated2DPosesArray, queue_size=1
        )

        # Setup publisher for output result image
        self.image_pub = rospy.Publisher(
            "~result_image", sensor_msgs.msg.Image, queue_size=1
        )

        # For visualization
        self.cam_helper = O2ACCameraHelper()
        self.listener = tf.TransformListener()
        self.pose_marker_id_counter = 0
        self.pose_marker_array = 0
        self._camera_info = None
        self._depth = None
        self.marker_array_pub = rospy.Publisher(
            "~result_marker_arrays", visualization_msgs.msg.MarkerArray, queue_size=1
        )

        rospy.loginfo("O2AC_vision has started up!")

    # ======= Callbacks of action servers

    def get_2d_poses_from_yolov5_goal_callback(self):
        print("get_2d_poses_from_yolov5 called")
        self.get_2d_poses_from_yolov5_server.accept_new_goal()

    def get_3d_poses_from_yolov5_goal_callback(self):
        rospy.loginfo('Callback 3D ....')
        self.get_3d_poses_from_yolov5_server.accept_new_goal()

    # ======= Callbacks of subscribed topics

    def synced_images_callback(self, camera_info, image, depth):
        self._camera_info = camera_info
        self._depth = depth
        rospy.loginfo_throttle(5, "synced images callback (alive)")

        im_in = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        im_vis = im_in.copy()

        # Pass image to SSD if continuous display is turned on
        if self.continuous_streaming:
            poses2d_array = o2ac_msgs.msg.Estimated2DPosesArray()
            poses2d_array.header = image.header
            poses2d_array.results, im_vis = self.get_2d_poses_from_yolov5(im_in, im_vis)
            self.results_pub.publish(poses2d_array)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

        elif self.get_2d_poses_from_yolov5_server.is_active():
            self.execute_get_2d_poses_from_yolov5(im_in, im_vis)

        elif self.get_3d_poses_from_yolov5_server.is_active():
            self.execute_get_3d_poses_from_yolov5(im_in, im_vis)

    # ======= Process active goals of action servers

    def execute_get_2d_poses_from_yolov5(self, im_in, im_vis):
        rospy.loginfo("Executing get_2d_poses_from_yolov5 action")

        action_result = o2ac_msgs.msg.get2DPosesFromSSDResult()
        action_result.results, im_vis = self.get_2d_poses_from_yolov5(im_in, im_vis)
        self.get_2d_poses_from_yolov5_server.set_succeeded(action_result)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

    def execute_get_3d_poses_from_yolov5(self, im_in, im_vis):
        rospy.loginfo("Executing get_3d_poses_from_yolov5 action")

        poses2d_array, im_vis = self.get_2d_poses_from_yolov5(im_in, im_vis)
        rospy.loginfo("pose 2d %s" % poses2d_array)
        action_result = o2ac_msgs.msg.get3DPosesFromSSDResult()
        success = True
        for poses2d in poses2d_array:
            for pose2d in poses2d.poses:
                p3d = self.convert_pose_2d_to_3d(pose2d)
                if p3d:
                    width, length = self.convert_bbox_to_3d_dimensions(poses2d.bbox)
                    pose3d_msg = o2ac_msgs.msg.Estimated3DPoses()
                    pose3d_msg.class_id = poses2d.class_id
                    pose3d_msg.pose = p3d
                    pose3d_msg.upside_down = poses2d.upside_down
                    pose3d_msg.confidence = poses2d.confidence
                    pose3d_msg.width = width
                    pose3d_msg.length = length
                    action_result.results.append(pose3d_msg)
        if not success:
            action_result = o2ac_msgs.msg.get3DPosesFromSSDResult()
            action_result.results = []
        self.get_3d_poses_from_yolov5_server.set_succeeded(action_result)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

    # ======= Localization helpers

    def get_2d_poses_from_yolov5(self, im_in, im_vis):
        """
        Finds an object's bounding box on the tray, then attempts to find its pose.
        Can also find grasp poses for the belt.

        Returns all bounding boxes with confidences, and returns all the 2D poses
        """
        # Object detection
        results, im_vis = self.detect_object_in_image(im_in, im_vis)
        print("yolo results:", results)

        self.reset_pose_markers()

        # Pose estimation
        poses2d_array = []  # Contains all results

        for result in results:

            target = int(result[6])
            # Stores the result for one item/class id
            poses2d = o2ac_msgs.msg.Estimated2DPoses()
            poses2d.class_id = target + 1
            poses2d.confidence = result[5]
            poses2d.bbox = result[:4]
            poses2d.upside_down = False

            rospy.loginfo("Seeing object id %d. Apply 3D pose estimation", target)
            x = int((poses2d.bbox[0] + poses2d.bbox[2]) / 2)
            y = int((poses2d.bbox[1] + poses2d.bbox[3]) / 2)
            pose2d = geometry_msgs.msg.Pose2D(
                x,
                y,
                result[4],  # theta
            )
            poses2d.poses = [pose2d]
            print("apply_2d_pose_estimation:", poses2d.poses)

            theta = -result[4]

            poses2d_array.append(poses2d)

            # Publish result markers
            poses3d = []
            for pose2d in poses2d.poses:
                pose2d.theta = theta
                pose3d = self.convert_pose_2d_to_3d(pose2d)
                if pose3d == CAMERA_FAILURE:
                    continue
                if pose3d:
                    poses3d.append(pose3d)
                    rospy.loginfo(
                        "Found pose for class %d: (%f, %f, %f)",
                        target,
                        pose3d.pose.position.x,
                        pose3d.pose.position.y,
                        pose3d.pose.position.z,
                    )

            if poses3d:
                self.add_markers_to_pose_array(poses3d)
                self.publish_stored_pose_markers()
            else:
                rospy.logwarn("Could not find pose for class %d!", target)

        return poses2d_array, im_vis

    def detect_object_in_image(self, im_in, im_vis):
        results, im_vis = self.yolov5_model.predict(im_in, im_vis)
        if results:
            return results[0], im_vis
        else:
            return [], im_vis

    def item_id(self, class_id):
        """Returns the name (item_id) of the item's id number (class_id) of the SSD."""
        return self._models[class_id - 1]

# ========

    def convert_bbox_to_3d_dimensions(self, bbox):
        rospy.loginfo("convert_bbox_to_3d_dimensions")
        if self._depth is None:
            rospy.logerr("No depth image found")
            return None
        depth = self.bridge.imgmsg_to_cv2(self._depth, desired_encoding="passthrough")
        x = int((bbox[0] + bbox[2]) / 2)
        y = int((bbox[1] + bbox[3]) / 2)
        # get depth for center
        depth_value = self.cam_helper.get_depth_value(self._camera_info, x, y, [depth])

        # project top left corner to 3d
        xy1 = self.cam_helper.project_2d_to_3d(self._camera_info, [bbox[0]], [bbox[1]], [depth_value])[0]
        # project bottom right corner to 3d
        xy2 = self.cam_helper.project_2d_to_3d(self._camera_info, [bbox[2]], [bbox[3]], [depth_value])[0]

        x_ = abs(xy1[0] - xy2[0])
        y_ = abs(xy1[1] - xy2[1])

        print("bbox", bbox)
        print("xy1", xy1, "xy2", xy2, x_, y_)

        width = x_ if x_ < y_ else y_
        length = y_ if x_ < y_ else x_

        return width, length

    def convert_pose_2d_to_3d(self, pose2d):
        """
        Convert a 2D pose to a 3D pose in the tray using the current depth image.
        Returns a PoseStamped in tray_center.
        """
        rospy.loginfo("convert_pose_2d_to_3d")
        p3d = geometry_msgs.msg.PoseStamped()
        p3d.header.frame_id = self._camera_info.header.frame_id
        if self._depth is None:
            rospy.logerr("No depth image found")
            return None
        depth = self.bridge.imgmsg_to_cv2(self._depth, desired_encoding="passthrough")

        depth_value = self.cam_helper.get_depth_value(self._camera_info, pose2d.x, pose2d.y, [depth])

        # Backproject to 3D (center)
        xyz = self.cam_helper.project_2d_to_3d(self._camera_info, [pose2d.x], [pose2d.y], [depth_value])[0]
        # We may not have found anything so it's okay to return None
        if not xyz:
            return None
        p3d.pose.position.x = xyz[0]
        p3d.pose.position.y = xyz[1]
        p3d.pose.position.z = xyz[2]

        # Transform position to tray_center
        try:
            p3d.header.stamp = rospy.Time(0.0)
            p3d = self.listener.transformPose("tray_center", p3d)
        except Exception as e:
            rospy.logerr("Pose transform failed(): {}".format(e))

        p3d.pose.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(
                0, tau / 4, -pose2d.theta + tau / 4
            )
        )
        rospy.loginfo("p3d %s" % p3d)
        rospy.loginfo("p3d theta %s" % pose2d.theta)
        return p3d

    def write_to_log(self, img_in, img_out, action_name):
        now = datetime.now()
        timeprefix = now.strftime("%Y-%m-%d_%H:%M:%S")
        folder = os.path.join(self.rospack.get_path("o2ac_vision"), "log")
        cv2.imwrite(
            os.path.join(folder, timeprefix + "_" + action_name + "_in.png"), img_in
        )
        cv2.imwrite(
            os.path.join(folder, timeprefix + "_" + action_name + "_out.png"), img_out
        )

    # ========  Visualization
    def clear_markers(self, namespace="/belt_grasp_poses"):
        delete_marker_array = visualization_msgs.msg.MarkerArray()
        del_marker = visualization_msgs.msg.Marker()
        del_marker.ns = namespace
        del_marker.action = del_marker.DELETEALL
        for i in range(1):
            dm = copy.deepcopy(del_marker)
            dm.id = i
            delete_marker_array.markers.append(dm)
        self.marker_array_pub.publish(delete_marker_array)

    def reset_pose_markers(self):
        self.clear_markers(namespace="/objects")
        self.pose_marker_id_counter = 0
        self.pose_marker_array = visualization_msgs.msg.MarkerArray()

    def publish_stored_pose_markers(self):
        self.marker_array_pub.publish(self.pose_marker_array)

    def add_markers_to_pose_array(self, poses_3d):
        """
        Add markers to the pose array.
        """
        i = self.pose_marker_id_counter
        for idx, viz_pose in enumerate(poses_3d):
            # print("treating pose: " + str(idx))
            # print(viz_pose.pose.position)
            # Set an auxiliary transform to display the arrows (otherwise they
            # are not rotated)
            base_frame_name = viz_pose.header.frame_id
            helper_frame_name = "viz_pose_helper_frame_" + str(idx)
            auxiliary_transform = geometry_msgs.msg.TransformStamped()
            auxiliary_transform.header.frame_id = base_frame_name
            auxiliary_transform.child_frame_id = helper_frame_name
            auxiliary_transform.transform.translation = geometry_msgs.msg.Vector3(
                viz_pose.pose.position.x,
                viz_pose.pose.position.y,
                viz_pose.pose.position.z,
            )
            auxiliary_transform.transform.rotation = viz_pose.pose.orientation
            self.listener.setTransform(auxiliary_transform)

            # Neutral PoseStamped
            p0 = geometry_msgs.msg.PoseStamped()
            p0.header.frame_id = helper_frame_name
            p0.header.stamp = rospy.Time(0.0)
            p0.pose.position = geometry_msgs.msg.Point(0, 0, 0)
            p0.pose.orientation = geometry_msgs.msg.Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            )

            # Draw arrows
            arrow_marker = visualization_msgs.msg.Marker()
            arrow_marker.ns = "/objects"
            arrow_marker.id = i
            arrow_marker.type = visualization_msgs.msg.Marker.ARROW
            arrow_marker.action = arrow_marker.ADD
            arrow_marker.header = viz_pose.header

            arrow_marker.color = std_msgs.msg.ColorRGBA(0, 0, 0, 0.8)
            arrow_marker.scale = geometry_msgs.msg.Vector3(0.05, 0.005, 0.005)

            i += 1
            arrow_x = copy.deepcopy(arrow_marker)
            arrow_x.id = i
            p = copy.deepcopy(p0)
            p.pose.orientation = geometry_msgs.msg.Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            )
            p = self.listener.transformPose(base_frame_name, p)
            arrow_x.pose = p.pose

            i += 1
            arrow_y = copy.deepcopy(arrow_marker)
            arrow_y.id = i
            p = copy.deepcopy(p0)
            p.pose.orientation = geometry_msgs.msg.Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(0, 0, tau / 4)
            )
            p = self.listener.transformPose(base_frame_name, p)
            arrow_y.pose = p.pose

            i += 1
            arrow_z = copy.deepcopy(arrow_marker)
            arrow_z.id = i
            p = copy.deepcopy(p0)
            p.pose.orientation = geometry_msgs.msg.Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(0, -tau / 4, 0)
            )
            p = self.listener.transformPose(base_frame_name, p)
            arrow_z.pose = p.pose

            arrow_x.color.r = 1.0
            arrow_y.color.g = 1.0
            arrow_z.color.b = 1.0

            self.pose_marker_array.markers.append(arrow_x)
            self.pose_marker_array.markers.append(arrow_y)
            self.pose_marker_array.markers.append(arrow_z)
        self.pose_marker_id_counter = i


if __name__ == "__main__":
    rospy.init_node("o2ac_vision_server", anonymous=False)
    c = O2ACVisionServer()
    rospy.spin()
