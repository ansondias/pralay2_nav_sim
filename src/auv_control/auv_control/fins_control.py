#!/usr/bin/env python3
"""Hardware-only propeller visualiser: integrates cmd_vel into a fake
T1_Joint position and publishes it on /joint_states so RViz shows the prop
turning when there is no joint feedback from the boards.

Do NOT run this under Gazebo: there joint_state_broadcaster publishes the
real joint states (gz_ros2_control) and this node would fight it. The sim
equivalent is cmd_vel_to_actuators.py, which commands the actual joints.
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class ThrusterVisualizer(Node):
    def __init__(self):
        super().__init__('thruster_visualizer')
        
        # REMOVED: self.thruster_pub is removed to stop the topic conflict!
        
        # Subscriber
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Internal state for position calculation
        self.thruster_pos = 0.0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info("Thruster visualizer node active (Read-Only mode).")

    def cmd_vel_callback(self, msg):
        # FIXED: Use the inverted simulation velocity so the visualization spins 
        # in the correct physical direction of the robot's movement.
        vel = -float(msg.linear.x)
        
        # Integrate Position for Visualization
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Update position: pos = pos + (vel * dt)
        self.thruster_pos += vel * dt
        self.thruster_pos = math.fmod(self.thruster_pos, 2 * math.pi)

        # Create and Publish JointState for visualization
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ['T1_Joint']  
        js_msg.position = [self.thruster_pos]
        js_msg.velocity = [vel]
        
        self.joint_state_pub.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()