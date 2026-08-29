# auv_planning

`nav2_core::GlobalPlanner` plugin(s) for the `planner_server`'s `GridBased`
slot. This is the package meant to be iterated on for custom path-planning
algorithms.

## AStarPlanner (`auv_planning::AStarPlanner`)

8-connected grid A* over the global costmap, with one twist: the cost of a
step is scaled by how well it aligns with a configured horizontal water
current, so paths lean toward current-assisted routes as current speed grows.

```
step_cost = |step| * (1 + costmap_cost/255) * current_multiplier(step)
current_multiplier = clamp(1 - tanh(speed) * current_cost_weight * alignment, 0.2, 2.0)
```

`alignment` is the dot product of the (unit) step direction with the current's
unit direction vector: `+1` = moving with the current (cheaper), `-1` = moving
straight into it (more expensive).

### Parameters (under `planner_server.ros__parameters.GridBased`)

| Parameter | Default | Meaning |
|---|---|---|
| `plugin` | - | `"auv_planning::AStarPlanner"` |
| `water_current_speed` | `0.0` | Current speed, m/s. `0` disables the current bias entirely. |
| `water_current_direction_north` | `0.0` | Compass bearing (deg, clockwise from north) the current flows *toward*. Same convention as `auv_current_plugin::OceanCurrent`'s `direction_north_deg`, so a scenario's numbers can be copied between the world file and this config. |
| `current_cost_weight` | `0.5` | How strongly current alignment affects step cost. `0` = ignore current. |
| `allow_unknown` | `true` | Whether unknown costmap cells are traversable. |

This is intentionally a simple, readable starting point (not tuned for
optimality) -- swap in your own search, sampling, or learned planner here as
long as it implements `nav2_core::GlobalPlanner` and reads the same two
`water_current_*` parameters, so it drops into the same nav2 config.
