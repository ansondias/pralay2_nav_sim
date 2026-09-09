# auv_planning

`auv_planning::AStarPlanner`, the `nav2_core::GlobalPlanner` behind the
`GridBased` slot in `auv_navigation/config/nav2_params.yaml`. (The Python
`a_star_planner.py` node is a standalone prototype with the same cost
conventions; Nav2 loads the C++ plugin.)

## Search

Textbook A* on the global costmap grid: 8-connected moves (no cutting a
diagonal between two blocked cells), g-scores per cell, closed set marked
when a node is expanded, stale queue entries skipped. Cells whose costmap
value is at or above `lethal_cost_threshold` (99) are impassable; with the
costmaps sized for the real hull (`robot_radius` 1.2, `inflation_radius`
2.0, `cost_scaling_factor` 1.5) that keeps every path about 1.8 m or more
from an obstacle edge, so the 2.41 m hull clears reefs mid-turn. Poses are
cell centres, the ends are the exact start/goal, and every pose carries a
heading along the path.

## Cost model

Each step costs an "equivalent distance" in metres:

```
step_cost = step_length * cruise / max(0.2*cruise, cruise + along*current*weight)
          + obstacle_cost_weight * resolution * cell_cost / lethal_cost_threshold
```

`along` is the cosine between the step direction and the current, so a
step against the current takes longer and costs more, a step with it
less (a travel-time model). The heuristic is the straight-line distance
scaled by the best-case multiplier, so it stays admissible and the search
optimal. The second term prefers the middle of a gap over its inflated
edges without closing the gap.

Parameters under `planner_server.GridBased`:

| parameter | default | meaning |
|---|---|---|
| `water_current_speed` | 0.3 | m/s, mirror of the sim's current (not wired automatically) |
| `water_current_direction_north` | 135 | bearing the current flows toward, degrees clockwise from north |
| `auv_cruise_speed` | 1.0 | m/s the current is compared against |
| `current_cost_weight` | 1.0 | 0 disables the current term |
| `obstacle_cost_weight` | 3.0 | inflated-cost penalty strength |
| `lethal_cost_threshold` | 99 | costmap value that counts as blocked |

## Smoothing

The raw grid path is sent to `smoother_server`'s `simple_smoother` **with
collision checking on**. Earlier it was off, and the smoother pulled every
corner toward the obstacle the path had just routed around. If smoothing
fails or is rejected the raw path is used.

Verified: from the spawn to (14, 8), a goal whose straight line crosses
reef_1, the plan kept 2.2 m from the reef edge, the hull passed north of
it untouched, and the goal succeeded without recoveries.
