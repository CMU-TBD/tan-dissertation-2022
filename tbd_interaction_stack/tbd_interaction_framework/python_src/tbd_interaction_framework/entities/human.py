"""
Copyright - Transporation, Bots, and Disability Lab - Carnegie Mellon University
Released under MIT License

human.py - Encodes a Human. The joints follow the Azure Kinect format.
"""

from enum import Enum

import numpy as np

from geometry_msgs.msg import (
    PoseStamped,
    Pose
)
import alloy
import alloy.math
import alloy.ros
import alloy.spatial.primitives


class HumanJointIds(Enum):
    """Enum for the types of joints in a human body.
    Follows the same convention as body tracking in Azure Kinect.
    """
    JOINT_PELVIS = 0
    JOINT_SPINE_NAVEL = 1
    JOINT_SPINE_CHEST = 2
    JOINT_NECK = 3
    JOINT_CLAVICLE_LEFT = 4
    JOINT_SHOULDER_LEFT = 5
    JOINT_ELBOW_LEFT = 6
    JOINT_WRIST_LEFT = 7
    JOINT_HAND_LEFT = 8
    JOINT_HANDTIP_LEFT = 9
    JOINT_THUMB_LEFT = 10
    JOINT_CLAVICLE_RIGHT = 11
    JOINT_SHOULDER_RIGHT = 12
    JOINT_ELBOW_RIGHT = 13
    JOINT_WRIST_RIGHT = 14
    JOINT_HAND_RIGHT = 15
    JOINT_HANDTIP_RIGHT = 16
    JOINT_THUMB_RIGHT = 17
    JOINT_HIP_LEFT = 18
    JOINT_KNEE_LEFT = 19
    JOINT_ANKLE_LEFT = 20
    JOINT_FOOT_LEFT = 21
    JOINT_HIP_RIGHT = 22
    JOINT_KNEE_RIGHT = 23
    JOINT_ANKLE_RIGHT = 24
    JOINT_FOOT_RIGHT = 25
    JOINT_HEAD = 26
    JOINT_NOSE = 27
    JOINT_EYE_LEFT = 28
    JOINT_EAR_LEFT = 29
    JOINT_EYE_RIGHT = 30
    JOINT_EAR_RIGHT = 31
    JOINT_COUNT = 32


class Human():
    """ Encodes a human in the scene.
    """

    id: int
    joints: dict

    base: np.array  # (4,4) A representative position of the agent in the world
    attention_ray: np.array  # (3, ) ray that describe where the human is looking at
    attention_origin: np.array  # (3,) that describe the origin of the attention
    focus_point: np.array  # (3, ) location of the human where if an agent is looking at should focus at.
    bounding_box: alloy.spatial.primitives.Box

    def __init__(self, human_msg=None):
        # if empty constructor
        if human_msg is None:
            return
        self.id = human_msg.body_id
        self.header = human_msg.header
        self.joints = {}
        # update all the joints & bounding box
        min_points = {
            'x': 1000,
            'y': 1000,
            'z': 0  # we assume that 'z' is always on the ground
        }
        max_points = {
            'x': -1000,
            'y': -1000,
            'z': -1000
        }

        for joint in human_msg.joints:
            self.joints[joint.joint_id] = joint.pose

            if joint.pose.position.x > max_points['x']:
                max_points['x'] = joint.pose.position.x
            if joint.pose.position.y > max_points['y']:
                max_points['y'] = joint.pose.position.y
            if joint.pose.position.z > max_points['z']:
                max_points['z'] = joint.pose.position.z

            if joint.pose.position.x < min_points['x']:
                min_points['x'] = joint.pose.position.x
            if joint.pose.position.y < min_points['y']:
                min_points['y'] = joint.pose.position.y
            if joint.pose.position.z < min_points['z']:
                min_points['z'] = joint.pose.position.z

        # caluclate new bounding box
        self.bounding_box = alloy.spatial.primitives.Box(list(min_points.values()), list(max_points.values()))

        # extrapolate the attention_ray by looking at the nose joint
        if HumanJointIds.JOINT_NOSE.value in self.joints:
            joint = self.joints[HumanJointIds.JOINT_NOSE.value]
            nose_transform_matrix = alloy.math.transformation_matrix_from_array(alloy.ros.pose_to_numpy(joint))
            # get the X-axis of the rotation matrix
            self.attention_ray = alloy.spatial.primitives.Ray(
                nose_transform_matrix[0:3, 3], nose_transform_matrix[0:3, 2])
        elif HumanJointIds.JOINT_CLAVICLE_LEFT in self.joints and HumanJointIds.JOINT_CLAVICLE_RIGHT in self.joints:
            left_joint_mat = alloy.math.transformation_matrix_from_array(
                alloy.ros.pose_to_numpy(self.joints[HumanJointIds.JOINT_CLAVICLE_LEFT.value]))
            right_joint_mat = alloy.math.transformation_matrix_from_array(
                alloy.ros.pose_to_numpy(self.joints[HumanJointIds.JOINT_CLAVICLE_RIGHT.value]))
            # get the average X-axis of them
            direction = np.linalg.norm(left_joint_mat[:3, 0] + -1 * right_joint_mat[:3, 0])
            origin = (left_joint_mat[:3, 3] + right_joint_mat[:3, 3])/2
            self.attention_ray = alloy.spatial.primitives.Ray(origin, direction)
        else:
            # we just going to use the pelvic center point
            joint = self.joints[HumanJointIds.JOINT_PELVIS.value]
            mat = alloy.math.transformation_matrix_from_array(alloy.ros.pose_to_numpy(joint))
            self.attention_ray = alloy.spatial.primitives.Ray(mat[0:3, 3], mat[0:3, 2])

        # we use the attention ray to figure out a base
        # Z-axis is always going up [0,0,1]
        self.base = np.eye(4)
        self.base[:3, 3] = self.attention_ray.origin
        self.base[:2, 0] = alloy.math.normalize(self.attention_ray.direction[:2])  # we collapse it down to (x,y)
        self.base[:3, 1] = np.cross(self.base[:3, 2], self.base[:3, 0])

        # try to figure out a focus point
        if HumanJointIds.JOINT_HEAD.value in self.joints:
            position = self.joints[HumanJointIds.JOINT_HEAD.value].position
            if HumanJointIds.JOINT_NOSE.value in self.joints:
                position = self.joints[HumanJointIds.JOINT_NOSE.value].position
            self.focus_point = np.array([position.x, position.y, position.z])
        else:
            # we take the pelvis and lookup 1 meter from it
            joint = self.joints[HumanJointIds.JOINT_NOSE.value]
            self.focus_point = np.array([joint.position.x, joint.position.y, joint.position.z + 1])

    def get_joint(self, joint_id: HumanJointIds) -> Pose:
        """Get the joint specified by the ID as a geometry_msgs/Pose

        Args:
            joint_id (HumanJointIds): Joint to return.

        Returns:
            Pose: The pose of the joint.
        """
        if joint_id.value in self.joints:
            return self.joints[joint_id.value]
        else:
            return None

    def get_joint_as_stamped_pose(self, joint_id: HumanJointIds) -> PoseStamped:
        """Get the joint specified by the ID as a geometry_msgs/PoseStamped

        Args:
            joint_id (HumanJointIds): Joint to return.

        Returns:
            PoseStamped: The stamped pose of the joint.
        """
        msg = PoseStamped()
        msg.pose = self.get_joint(joint_id)
        msg.header = self.header
        return msg

    def __str__(self):
        return f"human id:{self.id} with base at {self.base[:3,3]} looking at {self.base[:3,0]}"
