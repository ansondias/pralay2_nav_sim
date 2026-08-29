# pralay2_nav_sim

A standalone sandbox for testing Nav2 path planning against a simulated
water current, and for iterating on a custom global planner.
(a generic placeholder AUV, a fresh A* implementation, a custom lightweight
Gazebo current plugin). It exists to prototype ideas before they're ported
into the real `auv_planning` package in the main workspace.

Stack: ROS 2 Jazzy, Gazebo Harmonic (via `ros_gz`), the stock Nav2
navigation stack, `robot_localization` for sensor fusion.

## Quickstart

1. Install Docker and the VS Code "Dev Containers" extension.
   - **macOS**: just Docker Desktop -- no XQuartz/X11 setup needed, see
     "Viewing the GUI" below.
2. Open this folder in VS Code, then **"Reopen in Container"**.
   First build takes a while (pulling `osrf/ros:jazzy-desktop-full` +
   Nav2/Gazebo packages).
3. Inside the container terminal:
   ```bash
   cd /workspaces/pralay2_nav_sim
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch auv_bringup sim.launch.py
   ```
4. **Viewing the GUI**: the container runs a bundled virtual desktop
   (noVNC), not host X11 forwarding -- this is what makes "Reopen in
   Container" work the same way on Linux and macOS. Once the container is
   running, open **http://localhost:6080** in a browser (VS Code will also
   pop a "port forwarded" notification for 6080) to see the Gazebo and
   RViz windows. Rendering is software-only (`LIBGL_ALWAYS_SOFTWARE=1`),
   so it's not fast, but it's portable and needs no host GPU passthrough.
5. In RViz, use the "2D Goal Pose" (or Nav2 panel's goal tool) to send a
   goal and watch `auv_planning::AStarPlanner` plan a path across the
   `ocean_arena` map.

## Testing planning against a current

The current a scenario uses lives in two places that are **not** wired
together automatically -- update both by hand:

1. `src/auv_gazebo/worlds/ocean_arena.sdf` -- the `auv_current_plugin::OceanCurrent`
   plugin's `<speed>`/`<direction_north_deg>`, which is what actually pushes
   the simulated hull around.
2. `src/auv_navigation/config/nav2_params.yaml` -- `planner_server.GridBased`'s
   `water_current_speed`/`water_current_direction_north`, which is what the
   planner uses to bias its A* search.

Set both to the same values, rebuild (`colcon build`), relaunch, and compare
planned paths against `water_current_speed: 0.0` to see the effect. See
`src/auv_planning/README.md` for the cost model and
`src/auv_gazebo/README.md` for the plugin (including how to change the
current live over a Gazebo topic instead of editing the world file).

## Package layout

| Package | Purpose |
|---|---|
| `auv_planning` | **Main deliverable.** `nav2_core::GlobalPlanner` plugin(s) for the `GridBased` slot -- iterate on your planning algorithm here. |
| `auv_current_plugin` | Gazebo (gz-sim) system plugins: `OceanCurrent` (configurable horizontal current drag force) and `ThrustControl` (cmd_vel -> body-frame thrust force/torque, since a floating AUV has no ground contact for a wheeled-robot plugin to push against). |
| `auv_description` | Generic placeholder AUV URDF/xacro (torpedo hull, IMU, DVL-like odometry via gz-sim's `OdometryPublisher`). |
| `auv_gazebo` | The `ocean_arena` world, spawn launch, ros_gz bridge config. |
| `auv_localization` | `robot_localization` EKF (IMU + simulated DVL odom) -> `/odometry/filtered`. |
| `auv_navigation` | Nav2 params (stock defaults except the `GridBased` planner plugin) + a wrapper around `nav2_bringup`'s `navigation_launch.py`. |
| `auv_bringup` | Top-level launch tying everything together, plus the static obstacle map. |

## Known simplifications (by design, see the two accepted trade-offs above)

- **No buoyancy/hydrodynamics engine**: the AUV's gravity is disabled
  (`auv_description/urdf/auv.gazebo.xacro`) so it holds a fixed depth; only
  a horizontal current force is simulated. Swap in `gz-sim`'s `Buoyancy`
  system plugin or a full hydrodynamics stack if you outgrow this.
- **No onboard obstacle sensor**: the local costmap only inflates around the
  static map (`local_costmap` in `nav2_params.yaml` has no `obstacle_layer`).
  Add a sonar/lidar sensor to `auv_description` and wire it in for real
  obstacle avoidance beyond the static map. `collision_monitor`'s
  `observation_sources` points at `/unused_scan`, a topic nothing real
  publishes to -- `auv_bringup/scripts/unused_scan_publisher.py` sends a
  real but always-empty ("no detections") scan there instead of leaving
  it silent, because collision_monitor treats a source that never
  publishes as invalid and zeroes the robot's outgoing cmd_vel as a
  safety fallback (this is what silently blocked all Nav2-driven motion
  before that publisher was added -- manual `cmd_vel` still worked, since
  it bypasses collision_monitor entirely).
- **No global localization node**: `map -> odom` is a fixed identity
  transform (`auv_localization/launch/localization.launch.py`), not AMCL or
  a real DVL-based localizer. Fine for testing the planner/costmap/current
  interaction; replace it if you need to test localization too.
- Planning is effectively 2D (fixed depth) -- `water_current_direction_north`
  is a horizontal-plane bearing, matching that simplification.

## Regenerating the static map

If you move/add obstacles in `ocean_arena.sdf`, mirror the same coordinates
into `auv_bringup/scripts/generate_map.py`'s `OBSTACLES` list and re-run:

```bash
cd src/auv_bringup/scripts && python3 generate_map.py --out ../maps/ocean_arena
```
