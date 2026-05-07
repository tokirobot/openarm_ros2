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

    qos_arm = dora.Ros2QosPolicies(reliable=True)

    qos_cam = dora.Ros2QosPolicies(reliable=False)

    # --- 2. Define Publishers ---
    p_l_arm = node.create_publisher(node.create_topic(
        "/left_joint_trajectory_controller/joint_trajectory", "trajectory_msgs/JointTrajectory", qos_arm))
    p_r_arm = node.create_publisher(node.create_topic(
        "/right_joint_trajectory_controller/joint_trajectory", "trajectory_msgs/JointTrajectory", qos_arm))
    p_l_grp = node.create_publisher(node.create_topic(
        "/left_gripper_controller/joint_trajectory", "trajectory_msgs/JointTrajectory", qos_arm))
    p_r_grp = node.create_publisher(node.create_topic(
        "/right_gripper_controller/joint_trajectory", "trajectory_msgs/JointTrajectory", qos_arm))

    p_cam = node.create_publisher(node.create_topic(
        "/camera/image_raw/compressed", "sensor_msgs/CompressedImage", qos_cam))

    # --- 3. Pre-defined Constants & Templates (Object Reuse for Speed) ---
    EMPTY_F64 = np.array([], dtype=np.float64)
    SEC_0 = np.int32(0)
    NSEC_0 = np.uint32(0)
    NSEC_WAIT = np.uint32(20000000)  # 20ms

    NAMES_L_ARM = [f"openarm_left_joint{i+1}" for i in range(7)]
    NAMES_R_ARM = [f"openarm_right_joint{i+1}" for i in range(7)]
    NAMES_L_GRP = ["openarm_left_finger_joint1"]
    NAMES_R_GRP = ["openarm_right_finger_joint1"]

    joint_msg = {
        "header": {"stamp": {"sec": SEC_0, "nanosec": NSEC_0}, "frame_id": ""},
        "joint_names": None,
        "points": [{
            "positions": None,
            "velocities": EMPTY_F64,
            "accelerations": EMPTY_F64,
            "effort": EMPTY_F64,
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
    print("🚀 ROS 2 Bridge: All systems go (Joints + Compressed Image)", flush=True)

    for event in dora_node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        value = event["value"]

        # --- Case A: Camera Image (JPEG Pass-through) ---
        if eid == "camera_image":
            img_data = value.to_numpy().astype(np.uint8)
            compressed_img_msg["data"] = img_data
            p_cam.publish(pa.array([compressed_img_msg]))
            continue

        # --- Case B: Joint Positions ---
        vals = value.to_numpy().astype(np.float64)

        if eid == "left_position":
            pub_arm, pub_grp, name_arm, name_grp = p_l_arm, p_l_grp, NAMES_L_ARM, NAMES_L_GRP
        elif eid == "right_position":
            pub_arm, pub_grp, name_arm, name_grp = p_r_arm, p_r_grp, NAMES_R_ARM, NAMES_R_GRP
        else:
            continue

        joint_msg["joint_names"] = name_arm
        joint_msg["points"][0]["positions"] = vals[:7].tolist()
        pub_arm.publish(pa.array([joint_msg]))

        if len(vals) >= 8:
            joint_msg["joint_names"] = name_grp
            joint_msg["points"][0]["positions"] = vals[7:8].tolist()
            pub_grp.publish(pa.array([joint_msg]))


if __name__ == '__main__':
    main()
