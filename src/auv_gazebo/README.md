# auv_gazebo

`ocean_arena.sdf`: a bounded 40x40m arena (Gazebo / gz-sim, Harmonic) with 5
static reef obstacles and boundary walls, matching
`auv_bringup/maps/ocean_arena.yaml` (regenerate that map with
`auv_bringup/scripts/generate_map.py` if you move an obstacle).

## Water current

The AUV model loads `auv_current_plugin::OceanCurrent` (from the
`auv_current_plugin` package) as a `<gazebo>` plugin on the robot itself, in
`auv_description/urdf/auv.gazebo.xacro` (placeholder, `robot:=auv`) or
`ros_pralay_2_v1/urdf/ROS_PRALAY_2_V1.gazebo.xacro` (real hull, the
default) -- **not** in this world file. `spawn.launch.py`'s `robot` arg
picks the hull; both expose the same topics and frames. It
applies an anisotropic quadratic-drag horizontal force that pulls the hull
toward a configured current velocity -- see that package's header for the
model. The current now has three layers of environmental variation on top
of a flat scalar value:

- **Depth-dependent strength**: decays exponentially below an assumed free
  surface (`surface_z`), via `current_depth_decay_scale` -- an Ekman-layer
  style profile, strongest near the surface, weaker deeper down.
- **A monsoon cycle**: when `enable_monsoon_cycle` is true (the default),
  the current blends between an SW-monsoon and NE-monsoon vector over
  `monsoon_period_sec`, modelling the Indian Ocean's seasonal reversal --
  compressed to a period short enough to actually observe in one sim
  session, not real seasonal timescale. When false, the static
  `speed`/`direction_north_deg` below (live-settable, see below) is used
  instead.
- **Anisotropic drag**: separate `axial_*`/`lateral_*` drag
  coefficient/area, since a long thin hull resists broadside flow far more
  than nose-on flow. This also supplies real heave (vertical) damping.

It still does **not** model added mass, water density/temperature
stratification, or wave/tidal effects. Buoyancy, though, is real: see
`auv_current_plugin::Buoyancy` below -- the AUV's gravity is no longer
disabled, and `hull_mass` in `auv_description/urdf/auv.urdf.xacro` is
derived to be near-neutrally-buoyant by construction, not a fixed depth
hack. This is still the "lightweight" simulation tier relative to a full
hydrodynamics testbed (no added mass, no CFD), but it's no longer just a
horizontal, depth-blind current bias.

The plugin must be attached to the model, not the world: `OceanCurrent::Configure()`
requires a valid `gz::sim::Model(entity)` and silently no-ops otherwise, so
a copy of this plugin placed at world scope never applies any force. Set
the scenario's current in `auv_description/urdf/auv.gazebo.xacro`'s
`OceanCurrent` plugin block (see that file for every parameter), or change
just the manual-override fallback live while the sim is running (only takes
effect while `enable_monsoon_cycle` is false):

```bash
# <model name> is the spawn.launch.py `robot` arg: pralay2 (default) or auv
gz topic -t /model/pralay2/ocean_current -m gz.msgs.Vector3d -p "x: 0.8, y: 90.0"
```

(`x` = speed m/s, `y` = direction-toward, bearing degrees clockwise from
north; `z` unused.)

To actually see the planner react, mirror the current's *base* speed/
direction into `auv_navigation/config/nav2_params.yaml`'s
`planner_server.GridBased.water_current_speed` /
`water_current_direction_north` -- the sim and the planner are not wired
together automatically in this sandbox, so a scenario is "set the current
the sim applies" + "tell the planner what that current is", both by hand.
With depth decay and the monsoon cycle active, this is only ever an
approximation of what the sim is applying at a given moment/depth; the
planner has no matching depth or time model.

## Thrust, drift and heading

`auv_current_plugin::ThrustControl` turns `/cmd_vel` into a **surge-only**
force along body +x (a propeller can't push sideways) plus a yaw torque,
both applied at the vehicle's composite centre of mass (`center_of_mass`
parameter; for the multi-link Pralay-2 export that is 0.135 m aft of
base_link's origin, and applying forces anywhere else turned them into
pitch/yaw moments that tumbled the hull). Consequences worth knowing:

- With no `/cmd_vel` for `cmd_timeout` (1 s) the propeller is off and the
  hull **drifts with the current** (~0.3 m/s at the default spawn depth).
  Nav2 publishes explicit zeros while it is in control, so it is unaffected.
- A cross-current pushes the hull sideways while it drives; the path
  follower has to steer against it. That is the point of the sandbox.
- The yaw loop is stiff (`angular_gain` 20 N*m/(rad/s)): heading holds
  against the current's moments, and commanded yaw rates are tracked.
  Yaw rate is still clamped to `v / min_turning_radius` (no turning in
  place).

## Depth hold

Once gravity and buoyancy are both real, nothing else regulates depth, and
any sub-newton imbalance is an unbounded drift of a few mm/s (measured:
~1.5 mm/s with buoyancy matched to weight to four digits). So
`auv_current_plugin::ThrustControl` now carries a minimal depth autopilot
(`depth_hold`, on by default in both hulls' gazebo xacros): a critically
damped spring on world z toward a target that starts at the spawn depth
and integrates `cmd_vel.linear.z` (m/s, up positive, capped by
`max_vertical_speed`). An absolute setpoint, in metres **below the
surface** (`surface_z`, 5 m in `ocean_arena`), can be sent any time:

```bash
ros2 topic pub --once /depth_setpoint std_msgs/msg/Float64 "{data: 3.5}"   # -> z = 1.5
```

(bridged to `/model/<name>/depth_setpoint`, see `config/bridge.yaml`).
Vertical force is capped by `max_vertical_force`, separately from the
surge/yaw caps. Set `depth_hold` false to get the raw hydrostatics back.

## Buoyancy

`auv_current_plugin::Buoyancy`, also attached to the model in the hull's
gazebo xacro, applies a real upward buoyant force
(`fluid_density * hull_volume * g`, with `g` read from the world unless
`gravity_magnitude` overrides it) at a configurable `center_of_buoyancy`
(a link-frame point measured from the link origin). Putting that point a
few cm above the vehicle's centre of gravity gives the metacentric
righting moment -- passive pitch/roll stability, not just "floats at a
fixed height." Gravity is enabled (`<gravity>true</gravity>` on
`base_link`). For the placeholder, `hull_mass` is derived from
`hull_volume` at neutral density and the CoB is straight above the origin.
For the real Pralay-2 export (many links, tail-heavy), `hull_volume` is
sized from the *total* vehicle mass and the CoB sits above the composite
centre of gravity (x = -0.135 m); see `ros_pralay_2_v1/README.md`. The
force must act there: gz-sim applies wrenches at the link origin, and a
first version that did exactly that pitched the real hull ~80 deg nose-up.

Nav2 and the EKF (`auv_localization/config/ekf.yaml`, `two_d_mode: true`)
still deliberately ignore Z/roll/pitch -- this adds physical realism to the
sim, not 3D localization or planning. The vehicle can now genuinely pitch,
roll, and drift in depth; the navigation stack still only sees and acts on
X/Y/yaw, by design.
