# IK Bag Processor

A ROS2 package for processing ROS bags containing PoseStamped messages and computing inverse kinematics (IK) solutions using MoveIt.

## Features

- Reads PoseStamped messages from ROS bags (supports both SQLite3 and MCAP formats)
- Computes IK solutions using MoveIt's `/compute_ik` service
- Publishes resulting joint states to `/ik_joint_states` topic
- Provides debug output for monitoring IK success/failure
- Supports custom topic names for input poses
- Automatically detects bag storage format (SQLite3 or MCAP)

## Prerequisites

- ROS2 (tested with Humble and Jazzy)
- MoveIt2
- A running MoveIt setup with IK solver configured

## Installation

1. Build the package:
```bash
cd /path/to/your/workspace
colcon build --packages-select ik_bag_processor
source install/setup.bash
```

## Usage

### Method 1: Direct execution

```bash
# Basic usage with default topic name
ros2 run ik_bag_processor ik_processor /path/to/your/bag

# Specify custom topic name
ros2 run ik_bag_processor ik_processor /path/to/your/bag --topic /custom/pose/topic

# Using an MCAP file directly
ros2 run ik_bag_processor ik_processor /path/to/your/bag.mcap --topic /pose_stamped
```

### Method 2: Using launch file

```bash
# Launch with parameters
ros2 launch ik_bag_processor ik_processor.launch.py bag_path:=/path/to/your/bag topic:=/pose_stamped
```

## Parameters

- `bag_path` (required): Path to the ROS bag directory or MCAP file
- `topic` (optional, default: `/pose_stamped`): Topic name containing PoseStamped messages

## Supported Bag Formats

The package supports both SQLite3 and MCAP bag formats:

- **SQLite3 bags**: Standard ROS2 bag format, stored as a directory with `.db3` files
- **MCAP bags**: Modern bag format, can be either a directory with `.mcap` files or a single `.mcap` file

The storage format is automatically detected based on the file extension and directory contents.

## Output

The package will:
1. Read all PoseStamped messages from the specified topic in the bag
2. For each pose, call the MoveIt IK service
3. Publish successful IK solutions as JointState messages to `/ik_joint_states`
4. Print debug information including:
   - Detected bag format
   - Available topics in the bag
   - Number of poses processed
   - Number of successful IK solutions
   - Individual pose positions and joint values

## Example

If you have a bag with poses recorded at `/recorded_poses`, you can process it like this:

```bash
# Start your MoveIt setup first (in another terminal)
ros2 launch so_arm_moveit_config demo.launch.py

# Then process a SQLite3 bag
ros2 run ik_bag_processor ik_processor /path/to/recorded_poses

# Or process an MCAP bag
ros2 run ik_bag_processor ik_processor /path/to/recorded_poses.mcap
```

The script will output something like:
```
[INFO] [ik_processor]: Waiting for IK service...
[INFO] [ik_processor]: IK service available!
[INFO] [ik_processor]: Detected bag format: mcap
[INFO] [ik_processor]: Processing bag: /path/to/recorded_poses.mcap
[INFO] [ik_processor]: Looking for topic: /pose_stamped
[INFO] [ik_processor]: Available topics in bag:
[INFO] [ik_processor]:   - /pose_stamped (geometry_msgs/msg/PoseStamped)
[INFO] [ik_processor]: Processing pose 1: x: 0.02, y: -0.28, z: 0.27
[INFO] [ik_processor]: IK solution found!
[INFO] [ik_processor]: Published joint state: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
...
[INFO] [ik_processor]: Processing complete!
[INFO] [ik_processor]: Total poses processed: 10
[INFO] [ik_processor]: Successful IK solutions: 8
```

## Troubleshooting

1. **IK service not available**: Make sure MoveIt is running and the IK solver is properly configured
2. **No poses found**: Check that the topic name matches the one in your bag
3. **IK failures**: Some poses may not have valid IK solutions due to joint limits or collision constraints
4. **Bag format detection issues**: If the automatic format detection fails, try specifying the full path to the bag file or directory

## Next Steps

The joint states published to `/ik_joint_states` can be used to:
- Feed into a position controller
- Save to a new bag for later playback
- Analyze joint trajectories
- Validate motion planning 