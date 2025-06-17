#!/usr/bin/env python3

import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from typing import List, Tuple


def detect_storage_format(bag_path: str) -> str:
    """Detect the storage format of the bag file."""
    if os.path.isdir(bag_path):
        for file in os.listdir(bag_path):
            if file.endswith('.db3'):
                return 'sqlite3'
            if file.endswith('.mcap'):
                return 'mcap'
    elif os.path.isfile(bag_path) and bag_path.endswith('.mcap'):
        return 'mcap'
    return 'sqlite3'


def get_message_class(msg_type: str):
    """Get the ROS 2 message class from a message type string."""
    try:
        parts = msg_type.split('/')
        if len(parts) != 3:
            print(f"Invalid message type format: {msg_type}")
            return None

        package_name, msg_folder, msg_name = parts
        module_name = f"{package_name}.{msg_folder}"
        module = __import__(module_name, fromlist=[msg_name])
        return getattr(module, msg_name)
    except (ImportError, AttributeError) as e:
        print(f"Failed to get message class for {msg_type}: {e}")
        return None


def apply_transform(pose: PoseStamped, x_offset: float, y_offset: float, z_offset: float,
                    roll: float, pitch: float, yaw: float) -> PoseStamped:
    """
    Apply transform to a pose using the same logic as ik_processor.py.

    Args:
        pose: Input pose to transform
        x_offset, y_offset, z_offset: Translation offsets
        roll, pitch, yaw: Rotation angles in radians

    Returns:
        Transformed pose
    """
    # Create a new pose to avoid modifying the input
    transformed_pose = PoseStamped()
    transformed_pose.header = pose.header
    transformed_pose.header.frame_id = 'base'

    # Extract position and orientation
    p = pose.pose.position
    q = pose.pose.orientation

    # Convert to numpy arrays for easier computation
    pos = np.array([p.x, p.y, p.z])
    rot = np.array([q.x, q.y, q.z, q.w])

    # Create rotation matrix from Euler angles (ZYX order)
    # Roll (around X)
    cr = np.cos(roll)
    sr = np.sin(roll)
    Rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr]
    ])

    # Pitch (around Y)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    Ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp]
    ])

    # Yaw (around Z)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    Rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1]
    ])

    # Combine rotations in ZYX order (R = Rz * Ry * Rx)
    R = Rz @ Ry @ Rx

    # Apply rotation to position
    pos = R @ pos

    # Apply translation
    pos += np.array([x_offset, y_offset, z_offset])

    # Create rotation quaternion from Euler angles (ZYX order)
    # First create individual rotation quaternions
    # Roll (around X)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    q_roll = np.array([sr, 0.0, 0.0, cr])

    # Pitch (around Y)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    q_pitch = np.array([0.0, sp, 0.0, cp])

    # Yaw (around Z)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    q_yaw = np.array([0.0, 0.0, sy, cy])

    # Combine rotations in ZYX order (yaw * pitch * roll)
    # Quaternion multiplication helper function
    def quat_mult(q1, q2):
        return np.array([
            q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],  # x
            q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0],  # y
            q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3],  # z
            q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]   # w
        ])

    # Combine rotations: yaw * pitch * roll
    t_rot = quat_mult(quat_mult(q_yaw, q_pitch), q_roll)

    # Apply rotation to the original orientation
    rot = quat_mult(t_rot, rot)

    # Normalize quaternion
    rot = rot / np.linalg.norm(rot)

    # Update the transformed pose
    transformed_pose.pose.position.x = pos[0]
    transformed_pose.pose.position.y = pos[1]
    transformed_pose.pose.position.z = pos[2]
    transformed_pose.pose.orientation.x = rot[0]
    transformed_pose.pose.orientation.y = rot[1]
    transformed_pose.pose.orientation.z = rot[2]
    transformed_pose.pose.orientation.w = rot[3]

    return transformed_pose


def read_positions_from_bag(bag_path: str, topic_name: str = '/pose_stamped',
                            x_offset: float = 0.0, y_offset: float = 0.0, z_offset: float = 0.0,
                            roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Tuple[List[float], List[float], List[float], List[float]]:
    """
    Read position data from a ROS bag file.

    Args:
        bag_path: Path to the bag file
        topic_name: Name of the topic containing PoseStamped messages
        x_offset, y_offset, z_offset: Translation offsets
        roll, pitch, yaw: Rotation angles in radians

    Returns:
        Tuple containing lists of timestamps, x, y, and z coordinates
    """
    # Initialize lists to store data
    timestamps = []
    x_coords = []
    y_coords = []
    z_coords = []

    try:
        # Detect storage format
        storage_format = detect_storage_format(bag_path)
        print(f"Detected bag format: {storage_format}")

        # Open the bag
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id=storage_format
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        type_map = {
            topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        # Create message class map
        msg_class_map = {}
        for topic in topic_types:
            msg_class = get_message_class(topic.type)
            if msg_class is not None:
                msg_class_map[topic.name] = msg_class

        print(f"Processing bag: {bag_path}")
        print(f"Looking for topic: {topic_name}")

        # Print available topics
        print("Available topics in bag:")
        for topic in topic_types:
            print(f"  - {topic.name} ({topic.type})")

        # Read messages
        while reader.has_next():
            topic_name, data, t = reader.read_next()

            if topic_name == topic_name:
                try:
                    msg_class = msg_class_map.get(topic_name)
                    if msg_class is None:
                        print(f"No message class found for topic {topic_name}")
                        continue

                    pose_msg = deserialize_message(data, msg_class)

                    if isinstance(pose_msg, PoseStamped):
                        # Apply transform to the pose
                        transformed_pose = apply_transform(pose_msg, x_offset, y_offset, z_offset,
                                                           roll, pitch, yaw)

                        # Convert timestamp to seconds
                        timestamp = t / 1e9  # Convert nanoseconds to seconds

                        # Store position data from transformed pose
                        timestamps.append(timestamp)
                        x_coords.append(transformed_pose.pose.position.x)
                        y_coords.append(transformed_pose.pose.position.y)
                        z_coords.append(transformed_pose.pose.position.z)

                except Exception as e:
                    print(f"Error processing message: {e}")

        print(f"Read {len(timestamps)} position messages")

    except Exception as e:
        print(f"Error reading bag: {e}")

    return timestamps, x_coords, y_coords, z_coords


def plot_positions(timestamps: List[float], x_coords: List[float], y_coords: List[float], z_coords: List[float]):
    """Plot the position coordinates over time."""
    plt.figure(figsize=(12, 8))

    # Plot each coordinate
    plt.subplot(3, 1, 1)
    plt.plot(timestamps, x_coords, 'r-', label='X')
    plt.ylabel('X Position (m)')
    plt.title('Position Coordinates Over Time')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(timestamps, y_coords, 'g-', label='Y')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(timestamps, z_coords, 'b-', label='Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Z Position (m)')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Plot position coordinates from a ROS bag')
    parser.add_argument(
        'bag_path', help='Path to the ROS bag directory or MCAP file')
    parser.add_argument('--topic', default='/pose_stamped',
                        help='Topic name containing PoseStamped messages (default: /pose_stamped)')
    parser.add_argument('--x-offset', type=float, default=0.0,
                        help='X-axis translation offset (default: 0.0)')
    parser.add_argument('--y-offset', type=float, default=0.0,
                        help='Y-axis translation offset (default: 0.0)')
    parser.add_argument('--z-offset', type=float, default=0.0,
                        help='Z-axis translation offset (default: 0.0)')
    parser.add_argument('--roll', type=float, default=0.0,
                        help='Roll angle in radians (default: 0.0)')
    parser.add_argument('--pitch', type=float, default=0.0,
                        help='Pitch angle in radians (default: 0.0)')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='Yaw angle in radians (default: 0.0)')

    args = parser.parse_args()

    # Read position data from bag with transform parameters
    timestamps, x_coords, y_coords, z_coords = read_positions_from_bag(
        args.bag_path, args.topic,
        args.x_offset, args.y_offset, args.z_offset,
        args.roll, args.pitch, args.yaw
    )

    if timestamps:
        # Plot the data
        plot_positions(timestamps, x_coords, y_coords, z_coords)
    else:
        print("No position data found in the bag file.")


if __name__ == '__main__':
    main()
