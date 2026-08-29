# auv_gazebo

`ocean_arena.sdf`: a bounded 40x40m arena (Gazebo / gz-sim, Harmonic) with 5
static reef obstacles and boundary walls, matching
`auv_bringup/maps/ocean_arena.yaml` (regenerate that map with
`auv_bringup/scripts/generate_map.py` if you move an obstacle).

## Water current

The world loads `auv_current_plugin::OceanCurrent` (from the
`auv_current_plugin` package) as a world-level plugin targeting the AUV's
`base_link`. It applies a quadratic-drag horizontal force that pulls the hull
toward a configured current velocity -- see that package's header for the
model. It does **not** model buoyancy, added mass, or vertical currents; the
AUV's gravity is disabled in `auv_description` so it holds a fixed depth
(see `auv_description/urdf/auv.gazebo.xacro`). This is the "lightweight"
simulation tier: enough to see a planner react to currents, not a
hydrodynamics testbed.

Set the scenario's current in the world file:

```xml
<plugin filename="ocean_current" name="auv_current_plugin::OceanCurrent">
  <speed>0.4</speed>              <!-- m/s -->
  <direction_north_deg>135.0</direction_north_deg>
</plugin>
```

or change it live while the sim is running:

```bash
gz topic -t /model/auv/ocean_current -m gz.msgs.Vector3d -p "x: 0.8, y: 90.0"
```

(`x` = speed m/s, `y` = direction-toward, bearing degrees clockwise from
north; `z` unused.)

To actually see the planner react, mirror the same two numbers into
`auv_navigation/config/nav2_params.yaml`'s
`planner_server.GridBased.water_current_speed` /
`water_current_direction_north` -- the sim and the planner are not wired
together automatically in this sandbox, so a scenario is "set the current
the sim applies" + "tell the planner what that current is", both by hand.
