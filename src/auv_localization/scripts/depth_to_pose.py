#!/usr/bin/env python3
"""Turn the Bar30 depth into a pose the EKF can fuse.

robot_localization takes absolute position as a PoseWithCovarianceStamped,
not a bare Float64, so this node republishes

    /bar30/depth   std_msgs/Float64                     m below the surface
as
    /depth/pose    geometry_msgs/PoseWithCovarianceStamped   z = -depth, in odom

with only z meaningful (config/ekf.yaml fuses nothing else out of it).
The sign puts the surface at z = 0 and the hull below it, which is what
RViz then shows, since map->odom is the identity in this stack.

On the AUV the depth comes from depth_hold's pressure path (or the Bar30
driver) already zeroed at the surface; in Gazebo it is auv_control's
sim_depth_sensor. Either way the topic and units are the same.
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64


class DepthToPose(Node):
    def __init__(self):
        super().__init__("depth_to_pose")
        self.declare_parameter("depth_topic", "/bar30/depth")
        self.declare_parameter("pose_topic", "/depth/pose")
        self.declare_parameter("frame_id", "odom")
        # Bar30 is ~2 mm resolution but a few cm of wave/noise is realistic;
        # this is the 1-sigma the EKF weights the reading with.
        self.declare_parameter("depth_stddev", 0.02)

        self.frame_id = self.get_parameter("frame_id").value
        self.variance = float(self.get_parameter("depth_stddev").value) ** 2

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, 10
        )
        self.create_subscription(
            Float64, self.get_parameter("depth_topic").value, self.on_depth, 10
        )
        self.get_logger().info(
            f"{self.get_parameter('depth_topic').value} -> "
            f"{self.get_parameter('pose_topic').value} (z = -depth in {self.frame_id})"
        )

    def on_depth(self, msg: Float64):
        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.frame_id
        p.pose.pose.position.z = -float(msg.data)
        p.pose.pose.orientation.w = 1.0
        # Row-major 6x6 (x y z roll pitch yaw). Only z is fused; the rest is
        # marked as "know nothing" so a misconfigured filter can't lean on it.
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 6 + i] = 1e6
        cov[2 * 6 + 2] = self.variance
        p.pose.covariance = cov
        self.pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPose()
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
