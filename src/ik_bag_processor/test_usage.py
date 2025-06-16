#!/usr/bin/env python3

"""
Test script to demonstrate how to use the IK processor.
This script shows how to call the IK service directly without using ROS bags.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState


class IKTest(Node):
    """Simple test class to demonstrate IK computation."""

    def __init__(self):
        super().__init__('ik_test')

        # Create IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Wait for service
        self.get_logger().info("Waiting for IK service...")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("IK service not available, waiting...")

        self.get_logger().info("IK service available!")

    def test_ik(self, x: float, y: float, z: float):
        """Test IK computation for a given position."""

        # Create pose
        pose = PoseStamped()
        pose.header = Header(frame_id='base')
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Create IK request
        request = GetPositionIK.Request()

        ik_request = PositionIKRequest()
        ik_request.group_name = "arm"
        ik_request.ik_link_name = "gripper"
        ik_request.pose_stamped = pose
        ik_request.timeout = Duration(sec=2, nanosec=0)
        ik_request.avoid_collisions = False

        request.ik_request = ik_request

        # Call service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()

            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info(
                    f"IK solution found for position ({x}, {y}, {z}):")
                self.get_logger().info(
                    f"Joint positions: {response.solution.joint_state.position}")
                return response.solution.joint_state
            else:
                self.get_logger().warn(
                    f"IK failed for position ({x}, {y}, {z}) with error code: {response.error_code.val}")
                return None
        else:
            self.get_logger().error("IK service call timed out")
            return None


def main():
    """Main function to test the IK processor."""
    rclpy.init()

    try:
        test_node = IKTest()

        # Test with the position from your example
        test_node.get_logger().info("Testing IK with position from your example...")
        joint_state1 = test_node.test_ik(
            0.02061531359527537, -0.27747322066685803, 0.26685202775059064)

        # Test with an invalid position that should fail
        # x=0.08454592525959015, y=-0.025695502758026123, z=0.8113512992858887
        test_node.get_logger().info("Testing IK with invalid position...")
        joint_state2 = test_node.test_ik(
            0.08454592525959015, -0.025695502758026123, 0.8113512992858887)
        if joint_state2 is None:
            test_node.get_logger().info("Test passed - IK failed as expected for invalid position")
        else:
            test_node.get_logger().error("Test failed - IK succeeded when it should have failed")

        if joint_state1:
            test_node.get_logger().info("Test successful!")
        else:
            test_node.get_logger().error("Test failed!")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
