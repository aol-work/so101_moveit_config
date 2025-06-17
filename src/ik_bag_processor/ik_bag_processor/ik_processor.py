"""
This node reads PoseStamped messages from a ROS bag,
computes IK solutions using MoveIt's IK service, and publishes
the resulting joint states.
"""

import sys
import argparse
from typing import Optional, Tuple, Deque
from collections import deque
import os
import importlib

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool, Float64MultiArray
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class IKProcessor(Node):
    """
    A ROS2 node that reads PoseStamped messages from a ROS bag,
    computes IK solutions using MoveIt's IK service, and publishes
    the resulting joint states.
    """

    def __init__(self, bag_path: str, topic_name: str = "/pose_stamped",
                 transform_topic: Optional[str] = None,
                 x_offset: float = 0.0, y_offset: float = 0.0, z_offset: float = 0.0,
                 roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                 keep_running: bool = True, publish_to_position_controller: bool = False):
        super().__init__('ik_processor')

        self.bag_path = bag_path
        self.topic_name = topic_name
        self.keep_running = keep_running
        self.publish_to_position_controller = publish_to_position_controller

        # Store the last N poses for replay
        self.last_poses: Deque[Tuple[PoseStamped,
                                     JointState]] = deque(maxlen=100)

        # Initialize transform parameters
        self.transform = Transform()
        self.transform.translation.x = x_offset
        self.transform.translation.y = y_offset
        self.transform.translation.z = z_offset

        # Convert Euler angles to quaternion
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        self.transform.rotation.w = cr * cp * cy + sr * sp * sy
        self.transform.rotation.x = sr * cp * cy - cr * sp * sy
        self.transform.rotation.y = cr * sp * cy + sr * cp * sy
        self.transform.rotation.z = cr * cp * sy - sr * sp * cy

        # Create IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Create joint state publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/ik_joint_states',
            10
        )

        # Create position controller publisher if enabled
        if self.publish_to_position_controller:
            self.position_controller_pub = self.create_publisher(
                Float64MultiArray,
                '/position_controller/commands',
                10
            )
            self.get_logger().info("Position controller publishing enabled")

        # Create transform broadcaster for visualization
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create replay trigger subscriber
        self.replay_sub = self.create_subscription(
            Bool,
            '/ik_replay',
            self.replay_callback,
            10
        )

        # If transform topic is provided, subscribe to it
        if transform_topic:
            self.transform_sub = self.create_subscription(
                TransformStamped,
                transform_topic,
                self.transform_callback,
                10
            )
            self.get_logger().info(
                f"Listening for transforms on topic: {transform_topic}")

        # Wait for IK service to be available
        self.get_logger().info("Waiting for IK service...")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("IK service not available, waiting...")

        self.get_logger().info("IK service available!")
        self.get_logger().info(f"Initial transform: translation=({x_offset}, {y_offset}, {z_offset}), "
                               f"rotation=(roll={roll}, pitch={pitch}, yaw={yaw})")

        # Process the bag
        self.process_bag()

        if self.keep_running:
            self.get_logger().info("Node will keep running. Use these commands for debugging:")
            self.get_logger().info("  - To view joint states: ros2 topic echo /ik_joint_states")
            self.get_logger().info(
                "  - To replay last pose: ros2 topic pub /ik_replay std_msgs/msg/Bool 'data: true'")
            self.get_logger().info("  - To view transform: ros2 topic echo /tf")

    def replay_callback(self, msg: Bool):
        """Replay the last processed pose when triggered."""
        if msg.data and self.last_poses:
            pose, joint_state = self.last_poses[-1]
            self.get_logger().info("Replaying last pose...")

            # Apply transform and compute IK again
            transformed_pose = self.apply_transform(pose)
            new_joint_state = self.compute_ik(transformed_pose)

            if new_joint_state is not None:
                self.joint_state_pub.publish(new_joint_state)
                self.get_logger().info(
                    f"Published replayed joint state: {new_joint_state.position}")
            else:
                self.get_logger().warn("Failed to compute IK for replayed pose")

    def transform_callback(self, msg: TransformStamped):
        """Update the transform when a new one is received."""
        self.transform = msg.transform
        self.get_logger().info(f"Updated transform: translation=({msg.transform.translation.x}, "
                               f"{msg.transform.translation.y}, {msg.transform.translation.z})")

    def apply_transform(self, pose: PoseStamped) -> PoseStamped:
        """
        Apply the current transform to a pose.

        Args:
            pose: Input pose to transform

        Returns:
            Transformed pose
        """
        # Create a new pose to avoid modifying the input
        transformed_pose = PoseStamped()
        transformed_pose.header = pose.header
        # Ensure we're using the 'base' frame for IK
        transformed_pose.header.frame_id = 'base'

        # Extract position and orientation
        p = pose.pose.position
        q = pose.pose.orientation

        # Convert to numpy arrays for easier computation
        pos = np.array([p.x, p.y, p.z])
        rot = np.array([q.x, q.y, q.z, q.w])

        # Extract rotation angles from the transform
        x_offset = self.transform.translation.x
        y_offset = self.transform.translation.y
        z_offset = self.transform.translation.z

        # Convert quaternion to Euler angles (ZYX order)
        qx, qy, qz, qw = self.transform.rotation.x, self.transform.rotation.y, self.transform.rotation.z, self.transform.rotation.w

        # Roll (around X)
        roll = np.arctan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy))

        # Pitch (around Y)
        sinp = 2 * (qw*qy - qz*qx)
        pitch = np.arcsin(sinp) if abs(sinp) < 1 else np.sign(sinp) * np.pi/2

        # Yaw (around Z)
        yaw = np.arctan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))

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
                q1[3] * q2[0] + q1[0] * q2[3] +
                q1[1] * q2[2] - q1[2] * q2[1],  # x
                q1[3] * q2[1] - q1[0] * q2[2] +
                q1[1] * q2[3] + q1[2] * q2[0],  # y
                q1[3] * q2[2] + q1[0] * q2[1] -
                q1[1] * q2[0] + q1[2] * q2[3],  # z
                q1[3] * q2[3] - q1[0] * q2[0] -
                q1[1] * q2[1] - q1[2] * q2[2]   # w
            ])

        # Combine rotations: yaw * pitch * roll
        t_rot = quat_mult(quat_mult(q_yaw, q_pitch), q_roll)

        # Apply rotation to the original orientation
        rot = quat_mult(t_rot, rot)

        # Normalize quaternion
        rot = rot / np.linalg.norm(rot)

        # Log the transform being applied
        self.get_logger().info(
            f"Applying transform: translation=({x_offset}, {y_offset}, {z_offset}), "
            f"rotation=(roll={roll}, pitch={pitch}, yaw={yaw})")

        # Log the input position
        self.get_logger().info(
            f"Input position in {pose.header.frame_id} frame: ({p.x}, {p.y}, {p.z})")

        # Log the position after transformation
        self.get_logger().info(
            f"Position after transformation (in base frame): ({pos[0]}, {pos[1]}, {pos[2]})")

        # Update the transformed pose
        # TODO Extract command line parameters for the clamping
        transformed_pose.pose.position.x = min(
            0.3, max(-0.3, pos[0]))    # TODO Make configurable
        # TODO Make configurable
        transformed_pose.pose.position.y = min(-0.14, max(-0.16, pos[1]))
        transformed_pose.pose.position.z = pos[2]
        transformed_pose.pose.orientation.x = rot[0]
        transformed_pose.pose.orientation.y = rot[1]
        transformed_pose.pose.orientation.z = rot[2]
        transformed_pose.pose.orientation.w = rot[3]

        # Broadcast the transform for visualization
        t = TransformStamped()
        t.header = transformed_pose.header
        t.child_frame_id = "transformed_pose"
        t.transform = self.transform
        self.tf_broadcaster.sendTransform(t)

        return transformed_pose

    def detect_storage_format(self) -> str:
        """
        Detect the storage format of the bag file.

        Returns:
            str: The storage format ('sqlite3' or 'mcap')
        """
        # Check if the path is a directory (SQLite3) or a file (MCAP)
        if os.path.isdir(self.bag_path):
            # Check if it contains SQLite3 files
            for file in os.listdir(self.bag_path):
                if file.endswith('.db3'):
                    return 'sqlite3'

            # If no .db3 files found, check for MCAP files
            for file in os.listdir(self.bag_path):
                if file.endswith('.mcap'):
                    return 'mcap'
        elif os.path.isfile(self.bag_path) and self.bag_path.endswith('.mcap'):
            return 'mcap'

        # Default to SQLite3 if we can't determine
        return 'sqlite3'

    def get_message_class(self, msg_type: str):
        """
        Get the ROS 2 message class from a message type string.

        Args:
            msg_type: Message type string (e.g., 'geometry_msgs/msg/PoseStamped')

        Returns:
            Message class or None if not found
        """
        try:
            # Split the message type into package and message name
            parts = msg_type.split('/')
            if len(parts) != 3:
                self.get_logger().error(
                    f"Invalid message type format: {msg_type}")
                return None

            package_name, msg_folder, msg_name = parts

            # Import the module
            module_name = f"{package_name}.{msg_folder}"
            module = importlib.import_module(module_name)

            # Get the message class
            return getattr(module, msg_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(
                f"Failed to get message class for {msg_type}: {e}")
            return None

    def process_bag(self):
        """Read and process all PoseStamped messages from the ROS bag."""
        try:
            # Detect storage format
            storage_format = self.detect_storage_format()
            self.get_logger().info(f"Detected bag format: {storage_format}")

            # Open the bag
            storage_options = rosbag2_py.StorageOptions(
                uri=self.bag_path,
                storage_id=storage_format
            )

            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)

            topic_types = reader.get_all_topics_and_types()
            type_map = {
                topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

            # Create message class map
            msg_class_map = {}
            for topic in topic_types:
                msg_class = self.get_message_class(topic.type)
                if msg_class is not None:
                    msg_class_map[topic.name] = msg_class
                else:
                    self.get_logger().warn(
                        f"Could not get message class for topic {topic.name} with type {topic.type}")

            self.get_logger().info(f"Processing bag: {self.bag_path}")
            self.get_logger().info(f"Looking for topic: {self.topic_name}")

            # Print available topics for debugging
            self.get_logger().info("Available topics in bag:")
            for topic in topic_types:
                self.get_logger().info(f"  - {topic.name} ({topic.type})")

            pose_count = 0
            successful_ik_count = 0

            while reader.has_next():
                topic_name, data, t = reader.read_next()

                if topic_name == self.topic_name:
                    try:
                        # Get the message class for this topic
                        msg_class = msg_class_map.get(topic_name)
                        if msg_class is None:
                            self.get_logger().error(
                                f"No message class found for topic {topic_name}")
                            continue

                        # Deserialize the message
                        pose_msg = deserialize_message(data, msg_class)

                        if isinstance(pose_msg, PoseStamped):
                            pose_count += 1

                            # Log the frame_id of the input pose
                            self.get_logger().info(
                                f"Processing pose {pose_count} in frame: {pose_msg.header.frame_id}")

                            # Apply transform to the pose
                            transformed_pose = self.apply_transform(pose_msg)

                            self.get_logger().info(
                                f"Pose {pose_count} transform complete:\n"
                                f"  Original:    position=({pose_msg.pose.position.x}, {pose_msg.pose.position.y}, {pose_msg.pose.position.z})\n"
                                f"  Transformed: position=({transformed_pose.pose.position.x}, {transformed_pose.pose.position.y}, {transformed_pose.pose.position.z})")

                            # Compute IK with transformed pose
                            joint_state = self.compute_ik(transformed_pose)

                            if joint_state is not None:
                                successful_ik_count += 1
                                # Store the pose and joint state for replay
                                self.last_poses.append((pose_msg, joint_state))
                                # Publish joint state
                                self.joint_state_pub.publish(joint_state)
                                self.get_logger().info(
                                    f"Published joint state: {joint_state.position}")
                            else:
                                self.get_logger().warn(
                                    f"Failed to compute IK for pose {pose_count}")
                        else:
                            self.get_logger().warn(
                                f"Message is not a PoseStamped: {type(pose_msg)}")

                    except Exception as e:
                        self.get_logger().error(
                            f"Error processing message: {e}")

            self.get_logger().info(f"Processing complete!")
            self.get_logger().info(f"Total poses processed: {pose_count}")
            self.get_logger().info(
                f"Successful IK solutions: {successful_ik_count}")

        except Exception as e:
            self.get_logger().error(f"Error reading bag: {e}")

    def compute_ik(self, pose_stamped: PoseStamped) -> Optional[JointState]:
        """
        Compute IK for a given pose using MoveIt's IK service.

        Args:
            pose_stamped: The target pose for IK computation

        Returns:
            JointState if IK solution found, None otherwise
        """
        # Ensure we're using the base frame
        if pose_stamped.header.frame_id != 'base':
            self.get_logger().warn(
                f"Converting pose from {pose_stamped.header.frame_id} to base frame")
            pose_stamped.header.frame_id = 'base'

        # Log the exact pose being sent to IK
        self.get_logger().debug(
            f"Computing IK for pose in frame {pose_stamped.header.frame_id}:\n"
            f"  Position: ({pose_stamped.pose.position.x}, {pose_stamped.pose.position.y}, {pose_stamped.pose.position.z})\n"
            f"  Orientation: ({pose_stamped.pose.orientation.x}, {pose_stamped.pose.orientation.y}, "
            f"{pose_stamped.pose.orientation.z}, {pose_stamped.pose.orientation.w})")

        # Create IK request
        request = GetPositionIK.Request()

        # Set up the IK request
        ik_request = PositionIKRequest()
        ik_request.group_name = "arm"
        ik_request.ik_link_name = "gripper"
        ik_request.pose_stamped = pose_stamped
        ik_request.timeout = Duration(sec=2, nanosec=0)
        ik_request.avoid_collisions = False

        request.ik_request = ik_request

        # Log the complete IK request
        self.get_logger().info(
            f"IK request details:\n"
            f"  Group name: {ik_request.group_name}\n"
            f"  Link name: {ik_request.ik_link_name}\n"
            f"  Timeout: {ik_request.timeout.sec}.{ik_request.timeout.nanosec}\n"
            f"  Avoid collisions: {ik_request.avoid_collisions}")

        # Call the service
        future = self.ik_client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()

            # Check if IK was successful
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("IK solution found!")
                joint_state = response.solution.joint_state

                # If position controller publishing is enabled, publish the joint positions
                if self.publish_to_position_controller:
                    position_msg = Float64MultiArray()
                    position_msg.data = list(joint_state.position)
                    self.position_controller_pub.publish(position_msg)
                    self.get_logger().info(
                        f"Published to position controller: {position_msg.data}")

                return joint_state
            else:
                error_msg = f"IK failed with error code: {response.error_code.val}"
                try:
                    error_msg += f"\nError message: {MoveItErrorCodes.error_string(response.error_code.val)}"
                except AttributeError:
                    pass  # Ignore if error_string is not available
                self.get_logger().warn(error_msg)
                return None
        else:
            self.get_logger().error("IK service call timed out")
            return None


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Process ROS bag and compute IK solutions')
    parser.add_argument(
        'bag_path', help='Path to the ROS bag directory or MCAP file')
    parser.add_argument('--topic', default='/pose_stamped',
                        help='Topic name containing PoseStamped messages (default: /pose_stamped)')
    parser.add_argument('--transform-topic', default=None,
                        help='Topic to listen for dynamic transforms (default: None)')
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
    parser.add_argument('--exit-after-bag', action='store_true',
                        help='Exit after processing the bag (default: keep running)')
    parser.add_argument('--publish-to-position-controller', action='store_true',
                        help='Publish joint positions to /position_controller/commands (default: False)')

    # Parse command line arguments
    if args is None:
        args = sys.argv[1:]

    parsed_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        processor = IKProcessor(
            parsed_args.bag_path,
            parsed_args.topic,
            parsed_args.transform_topic,
            parsed_args.x_offset,
            parsed_args.y_offset,
            parsed_args.z_offset,
            parsed_args.roll,
            parsed_args.pitch,
            parsed_args.yaw,
            not parsed_args.exit_after_bag,
            parsed_args.publish_to_position_controller
        )

        if not parsed_args.exit_after_bag:
            rclpy.spin(processor)
        else:
            # Process the bag and exit
            processor.process_bag()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
