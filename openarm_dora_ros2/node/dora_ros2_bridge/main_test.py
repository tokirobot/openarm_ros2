#!/usr/bin/env python
import dora
import pyarrow as pa
import numpy as np
import sys


def main():
    # --- 1. ROS 2 Setup ---
    context = dora.Ros2Context()
    options = dora.Ros2NodeOptions(rosout=True)
    node = context.new_node("dora_to_ros2", "/openarm", options)

    # QoS Settings: Reliable for Joints, Best Effort for Cameras
    qos_arm = dora.Ros2QosPolicies(reliable=True)
    qos_cam = dora.Ros2QosPolicies(reliable=False)

    # --- 2. Define Publishers ---
    # Joint Publishers
    p_l_arm = node.create_publisher(node.create_topic(
        "/left_joint_trajectory_controller/joint_trajectory", "trajectory_msgs/JointTrajectory", qos_arm))
    p_r_arm = node.create_publisher(node.create_topic(
        "/right_joint_trajectory_controller/joint_trajectory", "trajectory_msgs/JointTrajectory", qos_arm))
    p_l_grp = node.create_publisher(node.create_topic(
        "/left_gripper_controller/joint_trajectory", "trajectory_msgs/JointTrajectory", qos_arm))
    p_r_grp = node.create_publisher(node.create_topic(
        "/right_gripper_controller/joint_trajectory", "trajectory_msgs/JointTrajectory", qos_arm))

    # Camera Publishers (5 Streams)
    cam_pubs = {
        "camera_wrist_right": node.create_publisher(node.create_topic("/camera_wrist_right", "sensor_msgs/CompressedImage", qos_cam)),
        "camera_wrist_left":  node.create_publisher(node.create_topic("/camera_wrist_left", "sensor_msgs/CompressedImage", qos_cam)),
        "camera_head_left":   node.create_publisher(node.create_topic("/camera_head_left", "sensor_msgs/CompressedImage", qos_cam)),
        "camera_head_right":  node.create_publisher(node.create_topic("/camera_head_right", "sensor_msgs/CompressedImage", qos_cam)),
        "camera_ceiling":     node.create_publisher(node.create_topic("/camera_ceiling", "sensor_msgs/CompressedImage", qos_cam)),
    }

    # --- 3. Schemas & Templates ---
    # PyArrow Schema to ensure correct data types and prevent buffer issues
    IMG_SCHEMA = pa.struct([
        pa.field("header", pa.struct([
            pa.field("stamp", pa.struct([
                pa.field("sec", pa.int32()),
                pa.field("nanosec", pa.uint32())
            ])),
            pa.field("frame_id", pa.string())
        ])),
        pa.field("format", pa.string()),
        pa.field("data", pa.list_(pa.uint8()))
    ])

    SEC_0, NSEC_0 = np.int32(0), np.uint32(0)
    NSEC_WAIT = np.uint32(20000000)

    NAMES_L_ARM = [f"openarm_left_joint{i+1}" for i in range(7)]
    NAMES_R_ARM = [f"openarm_right_joint{i+1}" for i in range(7)]
    NAMES_L_GRP, NAMES_R_GRP = ["openarm_left_finger_joint1"], [
        "openarm_right_finger_joint1"]

    joint_msg = {
        "header": {"stamp": {"sec": SEC_0, "nanosec": NSEC_0}, "frame_id": ""},
        "joint_names": None,
        "points": [{
            "positions": None,
            "velocities": np.array([], dtype=np.float64),
            "accelerations": np.array([], dtype=np.float64),
            "effort": np.array([], dtype=np.float64),
            "time_from_start": {"sec": SEC_0, "nanosec": NSEC_WAIT}
        }]
    }

    compressed_img_msg = {
        "header": {"stamp": {"sec": SEC_0, "nanosec": NSEC_0}, "frame_id": "world"},
        "format": "jpeg",
        "data": None
    }

    # --- 4. Dora Loop ---
    dora_node = dora.Node()
    print("🚀 ROS 2 Bridge: All systems go (5 Cameras + Joints)", flush=True)

    for event in dora_node:
        if event["type"] != "INPUT":
            continue

        eid, value = event["id"], event["value"]

        # --- Case A: Camera Images ---
        if eid in cam_pubs:
            # Convert to uint8 and copy to ensure memory safety
            img_data = value.to_numpy().astype(np.uint8).copy()
            compressed_img_msg["data"] = img_data

            # Publish to the specific camera topic using the mapped publisher
            cam_pubs[eid].publish(
                pa.array([compressed_img_msg], type=IMG_SCHEMA))
            continue

        # --- Case B: Joint Positions (IK results) ---
        if eid in ["left_position", "right_position"]:
            vals = value.to_numpy().astype(np.float64)

            if eid == "left_position":
                pub_arm, pub_grp, name_arm, name_grp = p_l_arm, p_l_grp, NAMES_L_ARM, NAMES_L_GRP
            else:
                pub_arm, pub_grp, name_arm, name_grp = p_r_arm, p_r_grp, NAMES_R_ARM, NAMES_R_GRP

            # Publish Arm joints (first 7)
            joint_msg["joint_names"] = name_arm
            joint_msg["points"][0]["positions"] = vals[:7]
            pub_arm.publish(pa.array([joint_msg]))

            # Publish Gripper joint (if exists)
            if len(vals) >= 8:
                joint_msg["joint_names"] = name_grp
                joint_msg["points"][0]["positions"] = vals[7:8]
                pub_grp.publish(pa.array([joint_msg]))


if __name__ == '__main__':
    main()
