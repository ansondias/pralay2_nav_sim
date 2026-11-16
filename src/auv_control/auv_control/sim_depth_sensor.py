#!/usr/bin/env python3
"""Stand-in for the Blue Robotics Bar30, for running the depth controller in Gazebo.

The simulator has no pressure sensor, so without this the depth-hold node
never gets a depth reading and sits in its "no data yet" state. This node
takes the hull's true vertical position out of the simulated odometry
(`/dvl_odom`, which the Pralay-2's gazebo xacro publishes in 3D) and
republishes it the way the real sensor and its driver would:

    /bar30/pressure   sensor_msgs/FluidPressure   Pa  (surface + rho*g*depth)
    /bar30/depth      std_msgs/Float64            m below the surface

`surface_z` is the world height of the free surface and must match the
`surface_z` in the hull's gazebo xacro (5.0 m in ocean_arena), because that
is the same reference `auv_current_plugin::ThrustControl` uses for its own
depth hold and for `/depth_setpoint`.

Only for simulation. On the AUV the real Bar30 driver publishes these
topics and this node is not started.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64


class SimDepthSensor(Node):
    def __init__(self):
        super().__init__("sim_depth_sensor")
        self.declare_parameter("odom_topic", "/dvl_odom")
        self.declare_parameter("surface_z", 5.0)
        self.declare_parameter("fluid_density", 1025.0)
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("surface_pressure_pa", 101325.0)
        # Bar30 noise is a few millibar; a little makes the filter and the
        # depth-rate term in the controller face something realistic.
        self.declare_parameter("noise_stddev_m", 0.01)

        self.surface_z = self.get_parameter("surface_z").value
        self.rho = self.get_parameter("fluid_density").value
        self.g = self.get_parameter("gravity").value
        self.p0 = self.get_parameter("surface_pressure_pa").value
        self.noise = self.get_parameter("noise_stddev_m").value

        self.pressure_pub = self.create_publisher(FluidPressure, "/bar30/pressure", 10)
        self.depth_pub = self.create_publisher(Float64, "/bar30/depth", 10)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10
        )
        self.get_logger().info(
            f"simulated Bar30: depth = {self.surface_z:.1f} m - odom z, "
            f"publishing /bar30/pressure and /bar30/depth"
        )

    def on_odom(self, msg: Odometry):
        import random

        depth = self.surface_z - msg.pose.pose.position.z
        if self.noise > 0.0:
            depth += random.gauss(0.0, self.noise)

        p = FluidPressure()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = "bar30"
        p.fluid_pressure = self.p0 + self.rho * self.g * depth
        p.variance = (self.rho * self.g * self.noise) ** 2
        self.pressure_pub.publish(p)

        self.depth_pub.publish(Float64(data=depth))


def main(args=None):
    rclpy.init(args=args)
    node = SimDepthSensor()
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
