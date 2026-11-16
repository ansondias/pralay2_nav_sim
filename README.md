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
   ros2 launch auv_bringup sim.launch.py            # real Pralay-2 hull (robot:=pralay2)
   ros2 launch auv_bringup sim.launch.py robot:=auv  # generic placeholder torpedo
   ```
   Existing containers need a **Rebuild Container** once after this pull:
   the Pralay-2 hull's fins/propeller run through `gz_ros2_control`, which
   the Dockerfile now installs.

   **Give Docker enough memory.** The headless stack alone (Gazebo server
   + EKF + Nav2) uses ~2.3 GB; the Gazebo GUI and RViz on software GL come
   on top, and the default Docker Desktop VM here is 3.8 GB. If Gazebo
   stalls or is killed, the spawn node waits forever, the EKF never gets
   odometry, and Nav2 loops on `Could not find a connection between
   'odom' and 'base_link'`. Set Docker Desktop > Settings > Resources >
   Memory to 8 GB (the devcontainer declares this in `hostRequirements`),
   or run `headless:=true rviz:=false` when you only need the planner.
   See `src/ros_pralay_2_v1/README.md` ("Troubleshooting").

   **If nodes freeze at start-up** (controller_server, the ros_gz bridge,
   the spawn node or RViz print nothing, controller spawners report
   "Failed to acquire lock", Ctrl-C needs SIGKILL): stale Fast DDS
   shared-memory files from earlier killed processes. Clear them with no
   ROS processes running:
   ```bash
   rm -f /dev/shm/fastrtps_*
   ```
   Sourcing `install/setup.bash` (after a build) also switches Fast DDS to
   UDP only via `auv_bringup/config/fastdds_no_shm.xml`, which avoids the
   problem entirely.
4. **Viewing the GUI**: the container runs a bundled virtual desktop
   (noVNC), not host X11 forwarding -- this is what makes "Reopen in
   Container" work http://localhost:6080 he same way on Linux and macOS. Once the container is
   running, open **** in a browser (VS Code will also
   pop a "port forwarded" notification for 6080) to see the Gazebo and
   RViz windows. Rendering is software-only (`LIBGL_ALWAYS_SOFTWARE=1`),
   so it's not fast, but it's portable and needs no host GPU passthrough.
5. In RViz, use the "2D Goal Pose" (or Nav2 panel's goal tool) to send a
   goal and watch `auv_planning::AStarPlanner` plan a path across the
   `ocean_arena` map.

## Mission sequencer (dive, transit, and optionally deliver)

`ros2 launch auv_bringup mission.launch.py` runs the whole system through a
twelve-phase sequencer, `auv_bringup/src/dive_and_navigate.cpp`. Goals come
from RViz's "2D Goal Pose" tool on `/mission/goal_pose`; watch progress on
`/mission/phase` and the result on `/mission/outcome`.

| # | Phase | Leaves when |
|---|---|---|
| 1 | DEPLOY | depth hold disabled, no thrust; the depth controller is seen |
| 2 | SENSOR_CHECK | depth (after the Bar30 surface zero), IMU and odometry all fresh; otherwise stays inert |
| 3 | PLAN | a goal arrives (until then the hull holds station: shuttles ±1 m about where the wait began with its bow settling into the current, and is steered back if the current still carries it `station_radius` off), Nav2 returns a drivable path and `V - C` clears `min_speed_margin_vs_current` on every leg; energy check runs only if a battery reading exists on `/battery/energy_wh` |
| 4 | ARM | depth hold enabled, holding the current depth (`require_arm_command` waits for `/mission/arm`) |
| 5 | CRUISE | measured speed reaches `min_speed_for_fins`, steering along the first leg |
| 6 | DIVE | depth enters the band |
| 7 | SETTLE | in band for `settle_time` |
| 8 | TRANSIT | Nav2 reaches the goal; depth hold owns the elevator throughout |
| 9-11 | DELIVERY_DEPTH, TERMINAL, DELIVER | only with `enable_delivery:=true`: re-target `delivery_depth`, close to `contact_range` at `terminal_speed`, publish `/payload/release`, wait for `/payload/released`, reset the depth integrator |
| 12 | END_STATE | outcome published, hull holds station as in PLAN (not after `on_abort: park`); a new goal continues from PLAN, a new `/mission/id` resets to DEPLOY |

Monitors run every cycle from phase 5: sensor freshness, `max_mission_depth`,
`abort_pitch`, energy against the remaining route, and live `V - C` toward
the goal. A trip aborts. The vehicle is one-way, so abort never retreats and
never releases: `on_abort:=hold` keeps depth, `on_abort:=park` switches
everything off.

Two things the simulator hides. Phase 5 exists because a fin-steered hull
cannot dive without forward speed; Gazebo's thrust plugin moves the hull
vertically on its own, so the dive would work in sim without it and fail on
the AUV. And the energy checks are skipped until something publishes
`/battery/energy_wh`; the sim has no battery.

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
| `ros_pralay_2_v1` | **Real Pralay-2 hull**: SolidWorks URDF export (fins, propeller, sensor links, meshes) + `sim_mode` xacro switch that adds the Gazebo plugins and `gz_ros2_control`. RViz-only and hardware launches too. |
| `auv_control` | ros2_control controllers for the Pralay-2 fins/propeller (same YAML on hardware and in Gazebo) and `cmd_vel_to_actuators`, which turns `/cmd_vel` into fin angles + propeller speed. |
| `auv_description` | Generic placeholder AUV URDF/xacro (torpedo hull, IMU, DVL-like odometry via gz-sim's `OdometryPublisher`). Kept as `robot:=auv`. |
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
