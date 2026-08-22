# Interface Information

The MoveIt2 capabilities package provides manipulator motion and state interfaces. Motion execution is delegated to the grasping stack, which owns workspace loading, planning-scene setup, and MoveIt planning configuration. State readers publish their results directly as event parameters.

## MoveToCartesianPoseRunner

This runner sends a Cartesian target through `grasping_msgs/action/MoveToPose`.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| frame_id | string | Frame of the requested target pose. Defaults to `base_link`. |
| x | double | Target position X. |
| y | double | Target position Y. |
| z | double | Target position Z. |
| qx | double | Target orientation quaternion X. |
| qy | double | Target orientation quaternion Y. |
| qz | double | Target orientation quaternion Z. |
| qw | double | Target orientation quaternion W. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| message | string | Final status returned by the grasping motion server. |

## MoveToJointPoseRunner

This runner sends an explicit joint-space target through `grasping_msgs/action/MoveToJointPose`. The runner passes only the target joint state; planning-group, planner, tolerance, and scene configuration are loaded by the grasping stack from its ROS parameters.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| joint_names | string[] | Ordered manipulator joint names. |
| joint_positions | double[] | Ordered target joint positions in radians matching `joint_names`. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| message | string | Final status returned by the grasping motion server. |

## MoveToNamedPoseRunner

This runner sends a configured named pose through `grasping_msgs/action/MoveToNamedPose`.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| pose_name | string | Name of a configured pose accepted by `motion_execution_node`. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| message | string | Final status returned by the grasping motion server. |

## CurrentCartesianPoseRunner

This runner reads the current transform between a configured parent and child frame and emits Cartesian plus roll-pitch-yaw values.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| parent_frame | string | TF parent frame. Defaults to `base_link`. |
| child_frame | string | TF child frame to query. Defaults to `tool0`. |
| timeout_sec | double | TF lookup timeout in seconds. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| x | double | Current position X. |
| y | double | Current position Y. |
| z | double | Current position Z. |
| qx | double | Current orientation quaternion X. |
| qy | double | Current orientation quaternion Y. |
| qz | double | Current orientation quaternion Z. |
| qw | double | Current orientation quaternion W. |
| roll | double | Current roll angle in radians. |
| pitch | double | Current pitch angle in radians. |
| yaw | double | Current yaw angle in radians. |

## CurrentJointPoseRunner

This runner reads the current manipulator joint state from `/joint_states` and emits vectors directly.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| joint_names | string[] | Optional ordered filter of joints to return. When omitted, all available planning joints are emitted. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| joint_names | string[] | Ordered joint names returned by the state reader. |
| joint_positions | double[] | Ordered joint positions in radians matching `joint_names`. |
| joint_count | int | Number of emitted joints. |