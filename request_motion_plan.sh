# You  need to have the move_group launched in another tmerinal

# Start the move_group node
ros2 launch so_arm_moveit_config move_group.launch.py

# Set sim time
ros2 param set /move_group use_sim_time true

# Do forward kinematics to get valid positions:
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: base}, fk_link_names: [gripper],
 robot_state: {joint_state: {name: [Rotation,Pitch,Elbow,Wrist_Pitch,Wrist_Roll],
                             position: [0.0,0.0,0.0,0.0,0.0]}}}"

# Check pose
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
"{ik_request: {group_name: arm, ik_link_name: gripper,
               pose_stamped: {header: {frame_id: base},
                              pose: {position: {x: 0.02061531359527537, y: -0.27747322066685803, z: 0.26685202775059064},
                                     orientation: {x: -0.49999951363112705, y: -0.5000036497428465, z: -0.4999949765360449, w: 0.5000018600477311}}},
               timeout: {sec: 2, nanosec: 0}}}"

# IK position only (set position_only_ik to true in kinematics.yaml, compile, re-source, restart move_group)
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{ik_request: {group_name: arm, ik_link_name: gripper,
               pose_stamped: {header: {frame_id: base},
                              pose: {position: {x: 0.02061531359527537, y: -0.27747322066685803, z: 0.26685202775059064}}},
               timeout: {sec: 2, nanosec: 0}}}"

# Send a position goal
ros2 action send_goal   /move_action   moveit_msgs/action/MoveGroup    --feedback --stdin < pose_goal.yaml
