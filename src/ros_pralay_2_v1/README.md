# ros_pralay_2_v1

SolidWorks URDF export of the real Pralay-2 hull (2.4 m x 0.16 m, 40 kg
all-up, as exported: four stern fins on revolute joints, one stern propeller
`T1_Joint`, DVL / IMU / GPS / depth-sensor / relay / nav-light links and
three dead-weight links on fixed joints), plus the Gazebo and RViz wiring
to run it in the `pralay2_nav_sim` sandbox instead of the placeholder
torpedo in `auv_description`.

The exported links, joints, meshes and inertials are untouched. Everything
sim-specific hangs off one xacro arg, `sim_mode`:

| `sim_mode` | used by | ros2_control hardware | base_link collision | extras |
|---|---|---|---|---|
| `false` (default) | `display.launch.py`, `robot.launch.py` (real AUV) | serial STM32 plugins or `mock_components` | 20 MB exported mesh | - |
| `true` | `auv_gazebo spawn.launch.py robot:=pralay2` | `gz_ros2_control/GazeboSimSystem` | 2.4 x r0.08 cylinder | `ROS_PRALAY_2_V1.gazebo.xacro` |

## Launch files

- `display.launch.py` - RViz + `joint_state_publisher_gui` sliders, no sim.
  `gui:=false` if something else publishes `/joint_states`.
- `gazebo.launch.py` - the hull in `auv_gazebo`'s `ocean_arena` world with
  RViz, controllers and the cmd_vel mapper, **no** Nav2. Args: `world`,
  `headless`, `rviz`, `x y z yaw`.
- `robot.launch.py` - real hardware only (reads `.env`, serial devices).

Full navigation stack: `ros2 launch auv_bringup sim.launch.py` (it now
defaults to `robot:=pralay2`; `robot:=auv` gets the old placeholder).

## What `ROS_PRALAY_2_V1.gazebo.xacro` attaches

All from packages that already exist in this workspace; nothing here is
Isaac-Sim specific any more.

| plugin | role |
|---|---|
| `gz_ros2_control::GazeboSimROS2ControlPlugin` | hosts the controller_manager inside gz-sim with `auv_control/config/controllers.yaml` -> fins (position) + propeller (velocity) + `/joint_states` |
| `auv_current_plugin::Buoyancy` | upward force sized from the *total* vehicle mass (11.3815 kg / 1025 kg/m^3) applied at the composite centre of gravity (x = -0.135 m) + 3 cm, so the tail-heavy multi-link hull floats level |
| `auv_current_plugin::ThrustControl` | `/cmd_vel` -> surge-only force + yaw torque at the composite CoM (this is what actually moves the hull); depth hold with `/depth_setpoint`; propeller off and free drift after 1 s without a command |
| `auv_current_plugin::OceanCurrent` | depth-decaying, monsoon-cycling current drag at the composite CoM; drag areas from the real hull envelope |
| `gz::sim::systems::OdometryPublisher` | `/dvl_odom` (DVL-like physics odometry, 3-D twist) |
| `gz::sim::systems::Imu` on link `IMU` | `/imu/data` |

Topics and frames match the placeholder exactly (`/cmd_vel`, `/dvl_odom`,
`/imu/data`, `base_link`, `odom`), so `auv_gazebo/config/bridge.yaml`,
`auv_localization/config/ekf.yaml` and `auv_navigation` are unchanged.

Kinematic limits in the xacro (`max_linear_speed` 2.06 m/s,
`min_turning_radius` 5.0 m) are deliberately kept equal to
`auv_navigation/config/nav2_params.yaml`. The URDF's +-52 deg fin travel
would allow a ~1.9 m radius; retune Nav2 and this file together if you
want it.

Verified in the devcontainer (headless, full `sim.launch.py`): model
spawns in ~2 s, `odom -> base_link` from the EKF is available ~3-12 s
after launch, hull holds roll/pitch within 0.05 rad in every manoeuvre,
heading holds with zero yaw command, `cmd_vel x: 1.0` gives ~0.95 m/s,
idle drift with the current ~0.3 m/s, fins track the rudder command and
the propeller spins, and a Nav2 goal 17 m away *behind* the vehicle
succeeded with a single wide U-turn and no recovery behaviours.

Nav2 is tuned for this hull in `auv_navigation/config/nav2_params.yaml`
(RPP lookahead 2-6 m, 2 m goal tolerance: with a 5.0 m turning radius a
missed tight goal means circling it) and uses
`auv_navigation/behavior_trees/navigate_to_pose_auv.xml`, the stock tree
minus the Spin recovery (it cannot rotate in place; the fins just sat at
full lock while Spin timed out).

## Meshes in Gazebo

The URDF uses `package://ros_pralay_2_v1/meshes/*.STL`. RViz resolves that
through ament; gz-sim needs `<install>/share` on `GZ_SIM_RESOURCE_PATH`
(sdformat rewrites the URIs to `model://ros_pralay_2_v1/...`).
`hooks/gz_sim_resource_path.dsv.in` adds it whenever the workspace is
sourced, and `auv_gazebo/launch/spawn.launch.py` appends it itself, so a
terminal sourced before this package was first built (symptom: `Unable to
find file with URI [model://ros_pralay_2_v1/meshes/...]`, robot invisible
in Gazebo while physics and TF work) still gets it.

`meshes/base_link_lowpoly.STL` is the hull visual used in `sim_mode`
(20k triangles instead of the export's 400k / 20 MB, which the software-GL
Gazebo GUI and RViz in the container struggle with). Regenerate after a
re-export with `pip install trimesh fast-simplification numpy` (Python
3.10+) and:

```python
import trimesh, fast_simplification, numpy as np
m = trimesh.load("meshes/base_link.STL", force="mesh")
v, f = fast_simplification.simplify(m.vertices, m.faces.astype(np.int64),
                                    target_reduction=0.95, agg=5)
trimesh.Trimesh(v, f, process=True).export("meshes/base_link_lowpoly.STL")
```

`export.log` and `urdf/ROS_PRALAY_2_V1/*.usd` are the SolidWorks/Isaac
export artefacts and aren't used by anything here.

## Troubleshooting: "Could not find a connection between 'odom' and 'base_link'"

`odom -> base_link` comes from the EKF (`auv_localization`), and
robot_localization publishes nothing until it has received a measurement
(`/dvl_odom`, `/imu/data`). Both come from the model in Gazebo through the
ros_gz bridge, so this message means **the model is not in the world (or
Gazebo is not stepping)**. Seeing it 3-6 times while Nav2's local costmap
activates is normal; seeing it continuously is not. Check, in order:

```bash
ros2 topic hz /clock        # Gazebo stepping + bridge alive
gz model --list             # should list pralay2 (or auv)
ros2 topic hz /dvl_odom     # OdometryPublisher plugin on the model
ros2 node list | grep ekf   # ekf_filter_node alive
```

Seen so far: a run where Gazebo stalled at start-up (launched one second
after a `colcon build`, GUI + RViz on a 3.8 GB VM) so `create` never
printed "Entity creation successful" and had to be Ctrl-C'd; and a stale
`install/` carrying the un-processable `ros2_control.urdf.xacro`
(whitespace before `<?xml`, fixed in source; a rebuild picks it up). If
`create` does not report success within ~10 s, restart the launch; use
`headless:=true rviz:=false` to cut memory when you only need Nav2.
