#!/usr/bin/env python
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
# Author: Karoly Istvan Artur, Felix von Drigalski, Cristian C. Beltran-Hernandez

import os
import yaml
import copy

import rospy
import rospkg
import tf

import geometry_msgs.msg
import moveit_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg

from ur_control import conversions

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    import pyassimp


class PartsReader(object):
    '''
    Loads collision objects with metadata from a database in the config folder, 
    for easy publishing to the MoveIt planning scene.
    The grasp data for each object can be published to the parameter server.
    '''

    def __init__(self, db_name="", load_meshes=True, verbose=True):
        self._rospack = rospkg.RosPack()
        self.db_name = db_name
        self.verbose = verbose
        if self.db_name:
            self.load_db(db_name, load_meshes=load_meshes)

    def load_db(self, db_name, load_meshes=True):
        '''
        Switch between assemblies
        '''
        if self.verbose:
            rospy.loginfo("Loading new parts database: " + db_name)
        self.db_name = db_name
        self._directory = os.path.join(self._rospack.get_path('o2ac_assembly_database'), 'config', db_name)
        self.assembly_info = self._read_assembly_info()
        self._parts_list = self._read_parts_list()
        self._collision_objects, self._grasps, self._mesh_filepaths, self._mesh_urls, self._collision_geometry = self.get_collision_objects_with_metadata()
        self.load_meshes = load_meshes
        if self.verbose:
            rospy.loginfo("Done loading parts database " + db_name)

    def get_collision_object(self, object_name, use_simplified_collision_shapes=True, scale=(0.001, 0.001, 0.001)):
        '''
        This returns the collision object (including subframes) for an object_name.

        If use_simplified_collision_shapes is true and primitives are defined in the object's metadata,
        they will be used as collision geometry instead of the object's mesh.
        '''
        for i, c_obj in enumerate(self._collision_objects):
            if c_obj.id == object_name:
                # Create copy to avoid modifying the original
                c_new = moveit_msgs.msg.CollisionObject()
                c_new.header = copy.deepcopy(c_obj.header)
                c_new.pose = copy.deepcopy(c_obj.pose)

                # Shallow copy the other fields to avoid deep copying meshes
                c_new.operation = c_obj.operation
                c_new.type = c_obj.type
                c_new.id = c_obj.id
                if self._collision_geometry[i] and use_simplified_collision_shapes:
                    rospy.loginfo("Using primitives for collision object: " + object_name)
                    c_new.primitives = self._collision_geometry[i][0]
                    c_new.primitive_poses = self._collision_geometry[i][1]
                # elif self.load_meshes and not c_obj.meshes:  # lazy loading of meshes
                else:  # lazy loading of meshes
                    rospy.loginfo("Loading meshes for collision object: " + object_name)
                    c_obj.meshes = [self._read_mesh(self._mesh_filepaths[i], scale)]
                    c_new.meshes = c_obj.meshes
                    c_new.mesh_poses = c_obj.mesh_poses
                # else:
                #     rospy.loginfo("Already loaded meshes for object " + object_name)
                #     c_new.meshes = c_obj.meshes
                #     c_new.mesh_poses = c_obj.mesh_poses

                # c_new.visual_geometry_mesh_url = self._mesh_urls[i]
                # try:
                #     c_new.visual_geometry_pose = c_obj.mesh_poses[0]
                #     # rospy.loginfo("Setting visual_geometry_pose of object " + c_obj.id)
                #     # print(c_new.visual_geometry_pose)
                # except Exception as e:
                #     print(e)

                c_new.planes = c_obj.planes
                c_new.plane_poses = c_obj.plane_poses
                c_new.subframe_names = c_obj.subframe_names
                c_new.subframe_poses = c_obj.subframe_poses
                return c_new
        rospy.logerr("Could not find collision object with id " + str(object_name))
        return None

    def get_visualization_marker(self, object_name, pose, frame_id, color=None, lifetime=0, frame_locked=False):
        """ Returns a visualization marker of the object on a given pose. """
        for (c_obj, mesh_filepath) in zip(self._collision_objects, self._mesh_filepaths):
            if c_obj.id == object_name:
                marker = visualization_msgs.msg.Marker()
                marker.id = self.name_to_id(object_name)
                marker.header.frame_id = frame_id
                marker.pose = pose
                marker.frame_locked = frame_locked
                marker.lifetime = rospy.Duration(lifetime)
                marker.type = marker.MESH_RESOURCE
                marker.mesh_resource = "file://" + mesh_filepath
                marker.scale = geometry_msgs.msg.Vector3(0.001, 0.001, 0.001)
                if color:
                    marker.color = color
                else:
                    marker.color.a = 1.0
                    marker.color.g = .8
                    marker.color.b = .5
                marker.action = marker.ADD
                return marker
        rospy.logerr("Could not find object with id " + str(object_name))
        return None

    def get_assembled_visualization_marker(self, object_name, marker_id_num=None):
        """ Returns a visualization marker of the object in its assembled position. Assumes the assembly frames have been published. """
        for (c_obj, mesh_filepath) in zip(self._collision_objects, self._mesh_filepaths):
            if c_obj.id == object_name:
                marker = visualization_msgs.msg.Marker()
                marker.type = marker.MESH_RESOURCE
                marker.mesh_resource = "file://" + mesh_filepath
                object_id = self.name_to_id(object_name)
                marker.id = marker_id_num if marker_id_num else object_id

                marker.header.frame_id = "assembled_part_" + str(object_id).zfill(2)  # Fill with leading zeroes
                marker.scale = geometry_msgs.msg.Vector3(0.001, 0.001, 0.001)
                try:
                    marker.pose = c_obj.mesh_poses[0]
                except:
                    marker.pose.orientation.w = 1.0

                marker.color.a = 1.0
                marker.color.g = .8
                marker.color.b = .5
                return marker
        rospy.logerr("Could not find object with id " + str(object_name))
        return None

    def get_grasp_pose(self, object_name, grasp_name):
        """
        Returns a geometry_msgs.msg.PoseStamped object in the frame of the object

        grasp_name is generally the format "grasp_0"
        """
        grasp_pose_stamped = geometry_msgs.msg.PoseStamped()
        grasp_pose_stamped.header.frame_id = object_name

        object_grasps = next((part for part in self._grasps if part["part_name"] == object_name), None)
        for (_grasp_name, grasp_pose) in zip(object_grasps['grasp_names'], object_grasps['grasp_poses']):
            if _grasp_name == grasp_name:
                grasp_pose_stamped.pose = grasp_pose
                return grasp_pose_stamped
        rospy.logerr("Did not find grasp_name " + grasp_name + " for object " + object_name)
        return None

    # Converters

    def id_to_name(self, id_num):
        """
        Returns the name of the object with the given id number.
        Returns an empty string on failure.
        """
        for obj in self._parts_list:
            if obj["id"] == id_num:
                return obj["name"]
        rospy.logerr("Could not find object with id " + str(id_num))
        return ""

    def name_to_id(self, name):
        """
        Returns the id of the object with the given name.
        Returns False on failure.
        """
        for obj in self._parts_list:
            if obj["name"] == name:
                return obj["id"]
        rospy.logerr("Could not find object with name " + str(name))
        return False

    def id_to_type(self, id_num):
        """
        Returns the type of the object with the given id number.
        Returns an empty string on failure.
        """
        for obj in self._parts_list:
            if obj["id"] == id_num:
                return obj["type"]
        rospy.logerr("Could not find object with id " + str(id_num))
        return ""

    def type_to_id(self, type):
        """
        Returns the id of the object with the given type.
        Returns False on failure.
        """
        for obj in self._parts_list:
            if obj["type"] == type:
                return obj["id"]
        rospy.logerr("Could not find object with type " + str(type))
        return False

    def name_to_type(self, name):
        """
        Returns the type of the object with the given name.
        Returns False on failure.
        """
        for obj in self._parts_list:
            if obj["name"] == name:
                return obj["type"]
        rospy.logerr("Could not find object with name " + str(name))
        return False

    ####

    def _upload_grasps_to_param_server(self, namespace):
        '''Upload grasps to the ROS param server
        Hierarchical params on the param server can be stored as dictionaries
        All of these grasps can be retrieved by requesting the parent parameter from the rosparam server
        '''
        for part in self._grasps:
            for (grasp_name, grasp_pose) in zip(part['grasp_names'], part['grasp_poses']):
                d = {'position': conversions.from_point(grasp_pose.position).tolist(),
                     'orientation': conversions.from_quaternion(grasp_pose.orientation).tolist()}
                param_name = '/'.join(['', namespace, part['part_name'], grasp_name])
                rospy.set_param(param_name, d)

    def _read_parts_list(self):
        path = os.path.join(self._directory, 'parts_list.yaml')
        with open(path, 'r') as file_open:
            parts_list = yaml.safe_load(file_open)
        return parts_list['parts_list']

    def _read_assembly_info(self):
        path = os.path.join(self._directory, 'assembly_info.yaml')
        with open(path, 'r') as file_open:
            assembly_info = yaml.safe_load(file_open)
        return assembly_info

    def _read_object_metadata(self, object_name):
        '''Read and return the object metadata including the subframes and the grasp points
        of the object referred to by input 'object_name'
        '''
        metadata_directory = os.path.join(self._directory, 'object_metadata')
        objects_with_metadata = [f.split('.')[0] for f in os.listdir(metadata_directory) if os.path.isfile(os.path.join(metadata_directory, f))]
        subframe_names = []
        subframe_poses = []
        grasp_names = []
        grasp_poses = []
        mesh_pose = []
        primitive_collision_objects = None
        if object_name in objects_with_metadata:
            with open(os.path.join(metadata_directory, object_name + '.yaml'), 'r') as f:
                data = yaml.safe_load(f)
            if "mesh_pose" in data:
                mesh_pose = geometry_msgs.msg.Pose()
                mesh_pose.position = conversions.to_vector3(conversions.to_float(data['mesh_pose'][0]['pose_xyzrpy'][:3]))
                mesh_pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(*conversions.to_float(data['mesh_pose'][0]['pose_xyzrpy'][3:])))
            subframes = data.get('subframes',  [])
            simplified_collision_object_text = data.get('collision_primitives', None)
            primitive_collision_objects = self._get_collision_shapes(simplified_collision_object_text)

            grasps = data['grasp_points']

            # Create the transformation between the world and the collision object
            transformer = tf.Transformer(True, rospy.Duration(10.0))
            collision_object_transform = geometry_msgs.msg.TransformStamped()
            collision_object_transform.header.frame_id = 'WORLD'
            collision_object_transform.child_frame_id = 'OBJECT'
            collision_object_transform.transform.rotation.w = 1
            transformer.setTransform(collision_object_transform)

            for subframe in subframes:
                subframe_names.append(subframe['name'])

                # Create the transformations between the collision object and the given subframe
                # Currently the subframes poses in the CollosionObject message are interpreted to be poses in the frame specified by the frame_id of the header of the CollosionObject
                # However, we want to define the subframe poses relative to the frame of the object, for ease of use
                # So we have to make this transformation to tell the pose of the subframes, that were defined in the object's frame, in the frame specified by the frame_if of the header of the CollosionObject
                subframe_transform = geometry_msgs.msg.TransformStamped()
                subframe_transform.header.frame_id = 'OBJECT'
                subframe_transform.child_frame_id = subframe['name']
                subframe_transform.transform.translation = conversions.to_vector3(conversions.to_float(subframe['pose_xyzrpy'][:3]))
                quaternion = tf.transformations.quaternion_from_euler(*conversions.to_float(subframe['pose_xyzrpy'][3:]))
                subframe_transform.transform.rotation = conversions.to_quaternion(quaternion)

                transformer.setTransform(subframe_transform)

                # Get the pose of the subframe in the world frame and add the subframe pose to the subframe poses of the collision object
                (trans, rot) = transformer.lookupTransform('WORLD', subframe['name'], rospy.Time(0))

                subframe_pose = conversions.to_pose(trans+rot)

                subframe_poses.append(subframe_pose)

            for grasp in grasps:
                grasp_names.append(grasp['grasp_name'])

                # Create the pose message for the grasp
                grasp_pose = geometry_msgs.msg.Pose()
                grasp_pose.position = conversions.to_vector3(conversions.to_float(grasp['pose_xyzrpy'][:3]))
                quaternion = tf.transformations.quaternion_from_euler(*conversions.to_float(grasp['pose_xyzrpy'][3:]))
                grasp_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion.tolist())

                grasp_poses.append(grasp_pose)

        else:
            if self.verbose:
                rospy.logwarn('Object \'' + object_name + '\' has no metadata defined! \n \
                            Returning empty metadata information! \n')
        return (subframe_names, subframe_poses, grasp_names, grasp_poses, mesh_pose, primitive_collision_objects)

    def _read_mesh(self, filename, scale):
        # Try to load the mesh and set it for the collision object
        if pyassimp is False:
            raise Exception("Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt")
        try:
            scene = pyassimp.load(filename)
        except Exception as e:
            rospy.logerr('Could not load mesh file ' + os.path.basename(os.path.normpath(filename)))
            print(">>>>> ", e)
            return None
        if not scene.meshes or len(scene.meshes) == 0:
            raise Exception("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise Exception("There are no faces in the mesh")

        mesh = shape_msgs.msg.Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, '__len__'):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = shape_msgs.msg.MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, 'indices'):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = shape_msgs.msg.MeshTriangle()
                    triangle.vertex_indices = [face.indices[0],
                                               face.indices[1],
                                               face.indices[2]]
                    mesh.triangles.append(triangle)
        else:
            raise Exception("Unable to build triangles from mesh due to mesh object structure")
        for vertex in scene.meshes[0].vertices:
            point = geometry_msgs.msg.Point()
            point.x = vertex[0]*scale[0]
            point.y = vertex[1]*scale[1]
            point.z = vertex[2]*scale[2]
            mesh.vertices.append(point)
        pyassimp.release(scene)
        return mesh

    def _get_collision_shapes(self, collision_objects):
        """ Convert YAML representation of collision primitive to the shape_msg.
        """
        if not collision_objects:
            return None

        PRIMITIVES = {"BOX": shape_msgs.msg.SolidPrimitive.BOX, "CYLINDER": shape_msgs.msg.SolidPrimitive.CYLINDER}
        primitives = []
        primitive_poses = []

        for co in collision_objects:
            primitive = shape_msgs.msg.SolidPrimitive()
            try:
                primitive.type = PRIMITIVES[co['type']]
            except KeyError as e:
                rospy.logerr("Invalid Collision Object Primitive type: %s " % co['type'])
                raise
            primitive.dimensions = co['dimensions']
            primitives.append(primitive)
            primitive_poses.append(conversions.to_pose(conversions.to_float(co['pose'])))

        return (primitives, primitive_poses)

    def _make_collision_object_from_mesh(self, name, pose, filename, scale=(1, 1, 1)):
        '''
        Reimplementation of '__make_mesh()' function from 'moveit_commander/planning_scene_interface.py'

        This function does not need to create a 'PlanningSceneInterface' object in order to work
        '''
        # Set the properties apart from the mesh
        co = self._make_collision_object_without_mesh(name, pose)
        co.meshes = [self._read_mesh(filename, scale)]
        return co

    def _make_collision_object_without_mesh(self, name, pose):
        '''
        Reimplementation of '__make_mesh()' function from 'moveit_commander/planning_scene_interface.py'

        This function does not need to create a 'PlanningSceneInterface' object in order to work
        '''
        # Set the properties apart from the mesh
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.pose.orientation.w = 1.0
        co.header = pose.header
        co.mesh_poses = [pose.pose]

        return co

    def get_collision_objects_with_metadata(self):
        '''
        Return a list of collision objects of the parts listed in 'parts list' along with their metadata (subframes, grasps)

        This function loops through the loaded 'parts list' and for each part
        it searches for the 3D representation (mesh) described in the 'cad' field
        of the 'parts list file', creates a collision object from the mesh and
        returns a list of such collision objects

        The returned collision objects are placed at the origin of the frame 'world'
        Use the MOVE option of the collision object once it is in the planning scene
        to move it to a different pose
        '''

        # List of collision objects to return
        collision_objects = []
        grasps = []
        mesh_filepaths = []
        mesh_urls = []
        primitive_collision_objects = []

        for part in self._parts_list:
            # Find mesh
            mesh_filepath = os.path.join(self._directory, 'meshes', part['cad'])
            mesh_url = "package://o2ac_assembly_database/config/" + self.db_name + "/meshes/" + part['cad']

            # Set the pose of the collision object
            posestamped = geometry_msgs.msg.PoseStamped()
            posestamped.header.frame_id = 'assembly_root'

            subframe_names, subframe_poses, grasp_names, grasp_poses, mesh_pose, primitive_collision_object = self._read_object_metadata(part['name'])

            collision_object_pose = geometry_msgs.msg.Pose()
            if mesh_pose:
                collision_object_pose = mesh_pose
            else:  # Neutral pose if no mesh pose has been defined
                collision_object_pose.orientation.w = 1

            posestamped.pose = collision_object_pose

            collision_object = self._make_collision_object_without_mesh(part['name'], posestamped)
            # collision_object = self._make_collision_object_from_mesh(part['name'], posestamped, mesh_filepath, scale=(0.001, 0.001, 0.001))

            collision_object.subframe_names = subframe_names
            collision_object.subframe_poses = subframe_poses

            grasps.append({'part_name': part['name'], 'grasp_names': grasp_names, 'grasp_poses': grasp_poses})

            # Here do we need the 'type' field of collision object (can be filled in from parts_list)?

            collision_objects.append(collision_object)

            mesh_filepaths.append(mesh_filepath)
            mesh_urls.append(mesh_url)

            primitive_collision_objects.append(primitive_collision_object)

        return (collision_objects, grasps, mesh_filepaths, mesh_urls, primitive_collision_objects)
