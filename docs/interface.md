# Interface Information

The grasping capabilities package provides manipulator motion and state interfaces. Motion execution is delegated to the grasping stack, which owns workspace loading, planning-scene setup, and MoveIt planning configuration. State readers publish their results directly as event parameters.

## ActivateSceneRunner

This runner sends an explicit scene reference through `grasping_msgs/action/ActivateScene`.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| scene_name | string | Optional registry-facing scene name attached to the reference. |
| package_name | string | ROS package that owns the workspace document. |
| workspace_file | string | Package-relative workspace document path to activate. |
| scene_revision | string | Optional source revision or content hash. |
| force_reload | bool | Reapply the scene even when the same reference is already active. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| success | bool | Whether the scene activation action reported success. |
| failure_reason | string | Structured failure code returned by the scene manager. |
| message | string | Final status returned by the scene manager. |
| active_scene_name | string | Active scene registry name after the request. |
| active_scene_package_name | string | Active scene source package name. |
| active_scene_workspace_file | string | Active scene workspace document path. |
| active_scene_revision | string | Active scene source revision. |
| scene_handle | string | Runtime scene handle returned by the scene manager. |
| workspace_base_frame | string | Base frame used by the activated workspace scene. |
| workspace_area_enabled | bool | Whether workspace-area filtering is enabled. |
| object_names | string[] | Collision object names loaded for the active scene. |

## MoveToCartesianPoseRunner

This runner sends a Cartesian target through `grasping_msgs/action/MoveToPose`.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| target_frame | string | The frame which reaches the given pose. This is not the planning frame. Defaults to `tcp`. |
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
| success | bool | Whether the grasping action reported a successful execution. |
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
| success | bool | Whether the grasping action reported a successful execution. |
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
| success | bool | Whether the grasping action reported a successful execution. |
| message | string | Final status returned by the grasping motion server. |

## ListNamedPosesRunner

This runner calls `grasping_msgs/srv/ListNamedPoses` and emits the configured pose names with their descriptions.

Input Parameters:

None

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| pose_names | string[] | Ordered configured named-pose identifiers. |
| descriptions | string[] | Ordered descriptions matching `pose_names`. |
| pose_count | int | Number of configured named poses. |

## CurrentCartesianPoseRunner

This runner reads the current transform between a configured parent and child frame and emits Cartesian plus roll-pitch-yaw values.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| from_frame | string | TF parent frame. Defaults to `base_link`. |
| to_frame | string | TF child frame to query. Defaults to `tcp`. |
| time_buffer | double | TF lookup timeout in seconds. |

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
| message | string | TF lookup error text when the transform cannot be resolved. |

## CurrentJointPoseRunner

This runner reads the current manipulator joint state from `/manipulator_joint_states` and emits vectors directly.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| timeout_sec | double | Maximum time in seconds to wait for the next planning joint-state sample. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| joint_names | string[] | Ordered joint names returned by the state reader. |
| joint_positions | double[] | Ordered joint positions in radians matching `joint_names`. |
| joint_count | int | Number of emitted joints. |
| message | string | Timeout error text when no planning joint-state sample arrives. |

## GetActiveSceneRunner

This runner calls `grasping_msgs/srv/GetActiveScene` and emits the current active scene metadata.

Input Parameters:

None

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| active | bool | Whether the scene manager currently has an active scene. |
| message | string | Human-readable response from the scene manager. |
| active_scene_name | string | Active scene registry name. |
| active_scene_package_name | string | Active scene source package name. |
| active_scene_workspace_file | string | Active scene workspace document path. |
| active_scene_revision | string | Active scene source revision. |
| scene_handle | string | Runtime scene handle for the active scene. |
| workspace_base_frame | string | Base frame used by the active scene. |
| workspace_area_enabled | bool | Whether workspace-area filtering is enabled. |
| object_names | string[] | Collision object names loaded in the active scene. |

## GetAvailableScenesRunner

This runner calls `grasping_msgs/srv/ListAvailableScenes` and emits flattened scene-descriptor fields.

Input Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| requested_mode | string | Optional mode filter forwarded to the scene registry. |
| required_tags | string[] | Optional tags that must all match for a scene to be returned. |

Output Parameters:

| Parameter Name | Type | Description |
| --- | --- | --- |
| message | string | Human-readable response from the scene-registry service. |
| scene_names | string[] | Ordered scene registry names matching the query. |
| package_names | string[] | Ordered package names matching `scene_names`. |
| workspace_files | string[] | Ordered workspace document paths matching `scene_names`. |
| scene_revisions | string[] | Ordered scene revisions matching `scene_names`. |
| station_frames | string[] | Ordered station frames matching `scene_names`. |
| selector_profiles | string[] | Ordered selector profiles matching `scene_names`. |
| descriptions | string[] | Ordered descriptions matching `scene_names`. |
| scene_count | int | Number of returned scene descriptors. |