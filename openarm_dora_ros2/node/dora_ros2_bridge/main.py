"""
Dora-to-ROS2 Bridge Node

This node acts as a state holder and message translator between the Dora dataflow
and ROS 2 control interfaces. It subscribes to joint position and camera inputs
from the Dora graph and forwards them to the appropriate ROS 2 controllers.
"""

#!/usr/bin/env python
import time
import dora
import pyarrow as pa
import numpy as np


def main():

    # --- 1. ROS 2 Setup ---
    context = dora.Ros2Context()
    options = dora.Ros2NodeOptions(rosout=True)
    node = context.new_node("dora_to_ros2", "/openarm", options)

    qos_arm = dora.Ros2QosPolicies(reliable=True)
    qos_cam = dora.Ros2QosPolicies(reliable=False)

    # --- 2. Define Publishers ---
    p_l_arm = node.create_publisher(node.create_topic(
        "/left_joint_trajectory_controller/joint_trajectory",
        "trajectory_msgs/JointTrajectory", qos_arm))
    p_r_arm = node.create_publisher(node.create_topic(
        "/right_joint_trajectory_controller/joint_trajectory",
        "trajectory_msgs/JointTrajectory", qos_arm))
    p_l_grp = node.create_publisher(node.create_topic(
        "/left_gripper_controller/joint_trajectory",
        "trajectory_msgs/JointTrajectory", qos_arm))
    p_r_grp = node.create_publisher(node.create_topic(
        "/right_gripper_controller/joint_trajectory",
        "trajectory_msgs/JointTrajectory", qos_arm))

    # Camera publishers
    CAMERA_TOPICS = {
        "camera_wrist_right": "/camera/wrist_right/image_raw/compressed",
        "camera_wrist_left":  "/camera/wrist_left/image_raw/compressed",
        "camera_head_left":   "/camera/head_left/image_raw/compressed",
        "camera_head_right":  "/camera/head_right/image_raw/compressed",
    }
    camera_publishers = {
        eid: node.create_publisher(
            node.create_topic(topic, "sensor_msgs/CompressedImage", qos_cam)
        )
        for eid, topic in CAMERA_TOPICS.items()
    }

    # --- 3. Pre-defined Constants ---
    EMPTY_F64   = np.array([], dtype=np.float64)
    STAMP_ZERO  = {"sec": np.int32(0), "nanosec": np.uint32(0)}

    NAMES_L_ARM = [f"openarm_left_joint{i+1}"  for i in range(7)]
    NAMES_R_ARM = [f"openarm_right_joint{i+1}" for i in range(7)]
    NAMES_L_GRP = ["openarm_left_finger_joint1"]
    NAMES_R_GRP = ["openarm_right_finger_joint1"]

    # --- 4. Helpers ---
    def now_stamp():
        t = time.time()
        return {"sec": np.int32(int(t)), "nanosec": np.uint32(int((t % 1.0) * 1e9))}

    def make_joint_msg(names, positions):
        return {
            "header": {"stamp": STAMP_ZERO, "frame_id": ""},
            "joint_names": names,
            "points": [{
                "positions": positions,
                "velocities": EMPTY_F64,
                "accelerations": EMPTY_F64,
                "effort": EMPTY_F64,
                "time_from_start": {"sec": np.int32(0), "nanosec": np.uint32(0)}
            }]
        }

    def make_compressed_img_msg(stamp, img_data):
        return {
            "header": {"stamp": stamp, "frame_id": "world"},
            "format": "jpeg",
            "data": img_data
        }

    # --- 5. Dora Loop ---
    dora_node = dora.Node()

    for event in dora_node:
        if event["type"] != "INPUT":
            continue

        eid   = event["id"]
        value = event["value"]

        # --- Case A: All Cameras (Already JPEG encoded) ---
        if eid in CAMERA_TOPICS:
            stamp = now_stamp()
            msg = make_compressed_img_msg(stamp, value.to_numpy().astype(np.uint8))
            camera_publishers[eid].publish(pa.array([msg]))
            continue

        # --- Case B: Joint Positions ---
        if eid in ("left_position", "right_position"):
            vals = value.to_numpy().astype(np.float64)

            if eid == "left_position":
                pub_arm, pub_grp, name_arm, name_grp = p_l_arm, p_l_grp, NAMES_L_ARM, NAMES_L_GRP
            else: # right_position
                pub_arm, pub_grp, name_arm, name_grp = p_r_arm, p_r_grp, NAMES_R_ARM, NAMES_R_GRP

            # if len(vals) >= 8:
                # vals[7] = 3.141592653589793/4 - vals[7]  # Invert gripper velocity for right arm

            pub_arm.publish(pa.array([make_joint_msg(name_arm, vals[:7].tolist())]))

            if len(vals) >= 8:
                pub_grp.publish(pa.array([make_joint_msg(name_grp, vals[7:8].tolist())]))


if __name__ == '__main__':
    main()