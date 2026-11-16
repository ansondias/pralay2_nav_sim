#!/usr/bin/env python3
"""Draw Nav2's plan at the depth it will actually be flown.

Nav2 plans in 2-D: every pose on /plan and /local_plan has z = 0, which in
this stack is the free surface. The hull follows that path at whatever
depth depth_hold is holding, so in RViz the path floats on the surface
while the robot model (now that the EKF is 3-D) sits a metre or more
below it. This node republishes each path with z = -depth so the two line
up:

    /plan        ->  /plan_at_depth          nav_msgs/Path
    /local_plan  ->  /local_plan_at_depth    nav_msgs/Path

Depth comes from depth_hold's target (`/depth_hold/target_depth`, latched
by the mission sequencer) by default, i.e. "the depth this plan will be
flown at"; set depth_source:=measured to use the current `/depth_hold/depth`
instead, which shows the path riding with the hull during a dive. Until
either arrives, default_depth is used.

Visualisation only: nothing reads these topics. The plan itself is still
2-D and depth is still owned by depth_hold; this just stops RViz lying
about where the path is.
"""

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


class PlanAtDepth(Node):
    def __init__(self):
        super().__init__("plan_at_depth")
        self.declare_parameter("input_topics", ["/plan", "/local_plan"])
        self.declare_parameter("suffix", "_at_depth")
        self.declare_parameter("depth_source", "target")   # target | measured
        self.declare_parameter("target_topic", "/depth_hold/target_depth")
        self.declare_parameter("measured_topic", "/depth_hold/depth")
        self.declare_parameter("default_depth", 1.0)

        self.depth = float(self.get_parameter("default_depth").value)
        source = self.get_parameter("depth_source").value
        if source == "measured":
            self.create_subscription(
                Float64, self.get_parameter("measured_topic").value, self.on_depth, 10
            )
        else:
            # The sequencer latches the target (transient_local); match it so
            # a late start still gets the current value.
            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                Float64, self.get_parameter("target_topic").value, self.on_depth, latched
            )

        suffix = self.get_parameter("suffix").value
        self.pubs = {}
        for topic in self.get_parameter("input_topics").value:
            out = topic + suffix
            self.pubs[topic] = self.create_publisher(Path, out, 10)
            self.create_subscription(
                Path, topic, lambda msg, t=topic: self.on_path(t, msg), 10
            )
            self.get_logger().info(f"{topic} -> {out}")
        self.get_logger().info(
            f"z = -{source} depth (starting at {self.depth:.2f} m)"
        )

    def on_depth(self, msg: Float64):
        self.depth = float(msg.data)

    def on_path(self, topic: str, msg: Path):
        z = -self.depth
        out = Path()
        out.header = msg.header
        out.poses = list(msg.poses)   # PoseStamped copies below, header shared
        for i, ps in enumerate(out.poses):
            ps2 = type(ps)()
            ps2.header = ps.header
            ps2.pose.position.x = ps.pose.position.x
            ps2.pose.position.y = ps.pose.position.y
            ps2.pose.position.z = z
            ps2.pose.orientation = ps.pose.orientation
            out.poses[i] = ps2
        self.pubs[topic].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PlanAtDepth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
