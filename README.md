# grasping_capabilities

Provides capabilities2 runners for manipulator motion and state queries. The package uses the grasping stack as the control boundary for Cartesian, named-pose, and joint-space execution, and reads state directly from TF and joint-state topics.

It also exposes scene-manager capabilities for explicit workspace activation and scene metadata queries.

## Supported runners

| Runner | Description |
| --- | --- |
| `MoveToCartesianPoseRunner` | Sends a Cartesian target through `grasping_msgs/action/MoveToPose`. |
| `MoveToJointPoseRunner` | Sends a joint-space target through `grasping_msgs/action/MoveToJointPose`. |
| `MoveToNamedPoseRunner` | Sends a configured named pose through `grasping_msgs/action/MoveToNamedPose`. |
| `ActivateSceneRunner` | Activates an explicit workspace scene reference through `grasping_msgs/action/ActivateScene`. |
| `ListNamedPosesRunner` | Lists configured named poses and their descriptions through `grasping_msgs/srv/ListNamedPoses`. |
| `CurrentCartesianPoseRunner` | Reads the current link pose from TF and emits Cartesian and RPY values. |
| `CurrentJointPoseRunner` | Reads the grasping-owned `/manipulator_joint_states` stream and emits `joint_names` plus `joint_positions` vectors. |
| `GetActiveSceneRunner` | Reads the currently active scene metadata through `grasping_msgs/srv/GetActiveScene`. |
| `GetAvailableScenesRunner` | Lists available scene descriptors through `grasping_msgs/srv/ListAvailableScenes`. |

## Planning Boundary

Motion execution is delegated to `grasping_control/motion_execution_node`, which owns:

- workspace loading into the MoveIt planning scene
- planner and tolerance configuration from `motion_config.yaml`
- TF normalization and nearby-IK behavior for pose requests

Because of that, `MoveToJointPoseRunner` now sends only the target joint state. Planning configuration is loaded on the grasping side rather than per capability trigger.

Named-pose discovery is also delegated to `grasping_control/motion_execution_node`, which reads the configured `poses_names` and each pose's `description` field from `motion_config.yaml`.

## Example Plans

- `plans/manipulation_1.xml`: one Cartesian move
- `plans/manipulation_2.xml`: one joint-space move
- `plans/manipulation_3.xml`: one named-pose move
- `plans/manipulation_4.xml`: four sequential named-pose moves

## Build

```bash
cd ~/colcon_ws
colcon build --packages-up-to grasping_msgs grasping_control grasping_capabilities --symlink-install
```

## Runtime

Start the capabilities server and the grasping motion server, then run one of the manipulation plans through your normal capabilities2 launch flow.