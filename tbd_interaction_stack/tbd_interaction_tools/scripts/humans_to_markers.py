#!/usr/bin/env python3
import random
import rospy

from tbd_ros_msgs.msg import (
    HumanBody,
    HumanBodyArray,
    HumanJoint
)

from geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    Vector3
)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker


JOINT_PAIR_LIST = [
    [HumanJoint.JOINT_PELVIS, HumanJoint.JOINT_HIP_LEFT],
    [HumanJoint.JOINT_PELVIS, HumanJoint.JOINT_HIP_RIGHT],
    [HumanJoint.JOINT_PELVIS, HumanJoint.JOINT_SPINE_NAVEL],
    [HumanJoint.JOINT_SPINE_NAVEL, HumanJoint.JOINT_SPINE_CHEST],
    [HumanJoint.JOINT_SPINE_CHEST, HumanJoint.JOINT_NECK],
    [HumanJoint.JOINT_NECK, HumanJoint.JOINT_HEAD],
    [HumanJoint.JOINT_HEAD, HumanJoint.JOINT_NOSE],
    [HumanJoint.JOINT_HEAD, HumanJoint.JOINT_EYE_LEFT],
    [HumanJoint.JOINT_HEAD, HumanJoint.JOINT_EAR_LEFT],
    [HumanJoint.JOINT_HEAD, HumanJoint.JOINT_EYE_RIGHT],
    [HumanJoint.JOINT_HEAD, HumanJoint.JOINT_EAR_RIGHT],
    # Left Leg
    [HumanJoint.JOINT_HIP_LEFT, HumanJoint.JOINT_KNEE_LEFT],
    [HumanJoint.JOINT_KNEE_LEFT, HumanJoint.JOINT_ANKLE_LEFT],
    [HumanJoint.JOINT_ANKLE_LEFT, HumanJoint.JOINT_FOOT_LEFT],
    # Right Lef
    [HumanJoint.JOINT_HIP_RIGHT, HumanJoint.JOINT_KNEE_RIGHT],
    [HumanJoint.JOINT_KNEE_RIGHT, HumanJoint.JOINT_ANKLE_RIGHT],
    [HumanJoint.JOINT_ANKLE_RIGHT, HumanJoint.JOINT_FOOT_RIGHT],
    # Left Arm
    [HumanJoint.JOINT_SPINE_CHEST, HumanJoint.JOINT_CLAVICLE_LEFT],
    [HumanJoint.JOINT_CLAVICLE_LEFT, HumanJoint.JOINT_SHOULDER_LEFT],
    [HumanJoint.JOINT_SHOULDER_LEFT, HumanJoint.JOINT_ELBOW_LEFT],
    [HumanJoint.JOINT_ELBOW_LEFT, HumanJoint.JOINT_WRIST_LEFT],
    [HumanJoint.JOINT_WRIST_LEFT, HumanJoint.JOINT_HAND_LEFT],
    [HumanJoint.JOINT_HAND_LEFT, HumanJoint.JOINT_HANDTIP_LEFT],
    [HumanJoint.JOINT_WRIST_LEFT, HumanJoint.JOINT_THUMB_LEFT],
    # Right Arm
    [HumanJoint.JOINT_SPINE_CHEST, HumanJoint.JOINT_CLAVICLE_RIGHT],
    [HumanJoint.JOINT_CLAVICLE_RIGHT, HumanJoint.JOINT_SHOULDER_RIGHT],
    [HumanJoint.JOINT_SHOULDER_RIGHT, HumanJoint.JOINT_ELBOW_RIGHT],
    [HumanJoint.JOINT_ELBOW_RIGHT, HumanJoint.JOINT_WRIST_RIGHT],
    [HumanJoint.JOINT_WRIST_RIGHT, HumanJoint.JOINT_HAND_RIGHT],
    [HumanJoint.JOINT_HAND_RIGHT, HumanJoint.JOINT_HANDTIP_RIGHT],
    [HumanJoint.JOINT_WRIST_RIGHT, HumanJoint.JOINT_THUMB_RIGHT],
]


class HumanToMarkers():



    def _humans_cb(self, msg: HumanBodyArray) -> None:

        marker_arr = MarkerArray()

        body: HumanBody
        for body in msg.bodies:
            # pick color
            if body.body_id not in self._color_dict:
                self._color_dict[body.body_id] = ColorRGBA(r=random.random(), g=random.random(), b=random.random(), a=1)
            color = self._color_dict[body.body_id]
            pose = Pose(orientation=Quaternion(w=1))
            # create new marker
            marker = Marker(header=body.header,type=Marker.LINE_LIST,pose=pose, action=0, id=body.body_id, color=color, scale=Vector3(x=0.05),lifetime=rospy.Duration(secs=5))
            # set the joints:
            joint_dict = dict(zip([joint.joint_id for joint in body.joints], [
                              joint.pose for joint in body.joints]))
            for pairs in JOINT_PAIR_LIST:
                if pairs[0] in joint_dict and pairs[1] in joint_dict:
                    point1 = joint_dict[pairs[0]].position
                    point2 = joint_dict[pairs[1]].position
                    marker.points.append(point1)
                    marker.points.append(point2)
            marker_arr.markers.append(marker)
        # publish marker
        self._marker_pub.publish(marker_arr)

    def __init__(self):
        self._color_dict = {}
        self._marker_pub = rospy.Publisher(
            'human_makers', MarkerArray, queue_size=1)
        rospy.Subscriber('humans', HumanBodyArray,
                         callback=self._humans_cb, queue_size=1)


if __name__ == "__main__":
    rospy.init_node("humans_to_markers")
    converter = HumanToMarkers()
    rospy.spin()
