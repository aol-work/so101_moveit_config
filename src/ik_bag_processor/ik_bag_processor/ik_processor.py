#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import rosbag2_py
from rclpy.serialization import deserialize_message
import sys
import argparse
from typing import List, Optional
import os
import importlib


class IKProcessor(Node):
    """
    A ROS2 node that reads PoseStamped messages from a ROS bag,
    computes IK solutions using MoveIt's IK service, and publishes
    the resulting joint states.
    """

    def __init__(self, bag_path: str, topic_name: str = "/pose_stamped"):
        super().__init__('ik_processor')

        self.bag_path = bag_path
        self.topic_name = topic_name

        # Create IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Create joint state publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/ik_joint_states',
            10
        )

        # Wait for IK service to be available
        self.get_logger().info("Waiting for IK service...")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("IK service not available, waiting...")

        self.get_logger().info("IK service available!")

        # Process the bag
        self.process_bag()

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
                            self.get_logger().info(
                                f"Processing pose {pose_count}: {pose_msg.pose.position}")

                            # Compute IK
                            joint_state = self.compute_ik(pose_msg)

                            if joint_state is not None:
                                successful_ik_count += 1
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

        # Call the service
        future = self.ik_client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()

            # Check if IK was successful
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("IK solution found!")
                return response.solution.joint_state
            else:
                self.get_logger().warn(
                    f"IK failed with error code: {response.error_code.val}")
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

    # Parse command line arguments
    if args is None:
        args = sys.argv[1:]

    parsed_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        processor = IKProcessor(parsed_args.bag_path, parsed_args.topic)
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
