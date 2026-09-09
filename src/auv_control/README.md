# auv_control

ros2_control side of the Pralay-2: the four fin joints (position) and the
stern propeller `T1_Joint` (velocity), as two `forward_command_controller`s
plus a `joint_state_broadcaster` (`config/controllers.yaml`). The joint
names come from `ros_pralay_2_v1/urdf/ros2_control.urdf.xacro`.

| | hardware / mock | Gazebo |
|---|---|---|
| launch | `controllers.launch.py` (default `sim:=false`) | `controllers.launch.py sim:=true` |
| controller_manager | `ros2_control_node` started here | hosted by `gz_ros2_control` inside gz-sim |
| hardware plugin | `propulsion_controller/...` or `mock_components` | `gz_ros2_control/GazeboSimSystem` |
| `T1_Joint` velocity units | -1..1 duty | rad/s |
| joint feedback | `fins_control` (fake, cmd_vel-integrated) | real, from physics |

Topics are identical in both cases:

```
/fin_controller/commands       std_msgs/Float64MultiArray  [Fin_Top, Fin_Left, Fin_Right, Fin_Bottom]  rad
/thruster_controller/commands  std_msgs/Float64MultiArray  [T1_Joint]
/joint_states                  sensor_msgs/JointState
```

## Nodes

- `cmd_vel_to_actuators` - `geometry_msgs/Twist` on `/cmd_vel` -> rudder
  angle (vertical fin pair, bicycle model `tan(d) = L*w/v`), elevator angle
  (horizontal pair, `atan2(vz, |vx|)`), propeller speed (`vx *
  thruster_scale`). Started automatically with `sim:=true`. Set
  `thruster_scale` to `1/max_speed` if you want to use it on hardware
  (`cmd_vel_bridge:=true`). Sign conventions and every parameter are
  documented at the top of `auv_control/cmd_vel_to_actuators.py`.
  `max_fin_angle` defaults to 0.80 rad, inside the URDF's 0.9076 limit, so
  the Gazebo position servo never fights the joint's hard stop.
- `depth_hold` (**C++**, `src/depth_hold.cpp`) - dives to `target_depth`
  (1.0 m) and holds it inside the 0.8..1.2 m band, from the Blue Robotics
  Bar30 pressure (auto-zeroed at the surface on start-up) and the BNO086
  pitch/pitch rate. Cascaded loop: depth error -> desired pitch (limited to
  20 deg) -> elevator angle (limited to 40 deg, inside the 45 deg servo
  travel), with fin authority scaled by speed. Publishes the elevator on
  `/depth_hold/elevator`, which `cmd_vel_to_actuators` mixes into the
  left/right fins while the rudder pair keeps following cmd_vel, so steering
  and depth hold coexist. Loses the depth sensor -> full nose-up (surface).
  `ros2 launch auv_control depth_hold.launch.py` (`config/depth_hold.yaml`);
  re-target with `/depth_hold/target_depth`, enable/disable with
  `/depth_hold/enable`, watch `/depth_hold/state` and `/depth_hold/in_band`.
  Gains, limits, signs and the target are also settable live with
  `ros2 param set /depth_hold <name> <value>`, for pool tuning without a
  restart. Under Gazebo pass `sim:=true`: the simulated fins produce no
  lift, so the node also forwards the target to the plugin depth hold. Sign
  parameters (`elevator_sign`, `pitch_sign`) exist for the first pool test.
- `depth_hold_py` (`auv_control/depth_hold.py`) - the original Python
  prototype of the node above: same parameters, same topics, same control
  law. Kept runnable for comparison; no launch file starts it.
- `fins_control` - hardware-only stand-in that synthesises `/joint_states`
  for the propeller from cmd_vel. Not for Gazebo.

Under Gazebo the hull is *moved* by `auv_current_plugin::ThrustControl`
(cmd_vel -> force), not by these joints; the fins/propeller follow the
command kinematically so RViz, the Gazebo GUI and `/joint_states` show what
the real actuators would be doing. gz-sim's `LiftDrag`/`Thruster` systems
are the path to closing that loop hydrodynamically if it's ever needed.

## Build layout

`auv_control` is an `ament_cmake` package (it was `ament_python` until the
depth-hold controller was ported to C++). `ament_cmake_python` still
installs the Python module and nodes under their original executable names,
so no launch file changed. `setup.py` / `setup.cfg` are left in the tree but
are no longer consulted: `CMakeLists.txt` is what defines the executables.
