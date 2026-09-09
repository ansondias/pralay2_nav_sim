#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException
from queue import PriorityQueue

class GraphNode:
    def __init__(self, x, y, cost=0, heuristic=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.prev = prev
    
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])

class AStarPlanner(Node):
    def __init__(self):
        super().__init__("a_star_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/costmap", self.map_callback, map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "/a_star/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/a_star/visited_map", 10)

        self.map_ = None
        self.visited_map_ = OccupancyGrid()

        # Water current at the AUV's fixed operating depth (1 m). This is a
        # single scalar current, not a depth-varying field: the planner only
        # ever sees a flat 2D costmap for that depth, so "the current" here
        # means "the current at 1 m," set once via these params -- it cannot
        # change with depth or position without a different map/costmap per
        # depth band.
        self.declare_parameter("water_current_speed", 0.0)
        self.declare_parameter("water_current_direction_north", 0.0)
        self.declare_parameter("current_cost_weight", 0.5)
        self.water_current_speed = self.get_parameter("water_current_speed").value
        self.water_current_direction_north = self.get_parameter(
            "water_current_direction_north"
        ).value
        self.current_cost_weight = self.get_parameter("current_cost_weight").value
        self.get_logger().info(
            f"AStarPlanner: water_current_speed={self.water_current_speed:.2f} m/s, "
            f"water_current_direction_north={self.water_current_direction_north:.1f} deg, "
            f"current_cost_weight={self.current_cost_weight:.2f}"
        )

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().error("No map received!")
            return

        self.visited_map_.data = [-1] * (self.visited_map_.info.height * self.visited_map_.info.width)

        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint", rclpy.time.Time()
            )
        except LookupException:
            self.get_logger().error("Could not transform from map to base_footprint")
            return

        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation

        path = self.plan(map_to_base_pose, pose.pose)
        if path.poses:
            self.get_logger().info("Shortest path found!")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start: Pose, goal: Pose):
        # Define possible movement directions
        explore_directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        # Priority queue with custom comparison for A* based on cost + heuristic
        pending_nodes = PriorityQueue()
        visited_nodes = set()

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)
        start_node.heuristic = self.manhattan_distance(start_node, goal_node)
        pending_nodes.put(start_node)

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()

            # Goal found!
            if active_node == goal_node:
                break
            
            # Explore neighbors
            for dir_x, dir_y in explore_directions:
                new_node: GraphNode = active_node + (dir_x, dir_y)
                
                if (new_node not in visited_nodes and self.pose_on_map(new_node) and 
                    0 <= self.map_.data[self.pose_to_cell(new_node)] < 99):
                    
                    step_cost = self.current_cost_multiplier(dir_x, dir_y)
                    new_node.cost = active_node.cost + step_cost + self.map_.data[self.pose_to_cell(new_node)]
                    new_node.heuristic = self.manhattan_distance(new_node, goal_node)
                    new_node.prev = active_node

                    pending_nodes.put(new_node)
                    visited_nodes.add(new_node)

            self.visited_map_.data[self.pose_to_cell(active_node)] = -106
            self.map_pub.publish(self.visited_map_)

        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        while active_node and active_node.prev and rclpy.ok():
            last_pose: Pose = self.grid_to_world(active_node)
            last_pose_stamped = PoseStamped()
            last_pose_stamped.header.frame_id = self.map_.header.frame_id
            last_pose_stamped.pose = last_pose
            path.poses.append(last_pose_stamped)
            active_node = active_node.prev

        path.poses.reverse()
        return path

    def current_cost_multiplier(self, dx, dy):
        """Cost scale for a step (dx, dy): <1 with the 1 m-depth current, >1 against it."""
        if self.water_current_speed <= 0.0 or (dx == 0 and dy == 0):
            return 1.0

        # World frame is ENU (+X east, +Y north); direction_north is the
        # compass bearing the current flows toward, same convention as the
        # C++ AStarPlanner / auv_current_plugin so params can be shared.
        theta = math.radians(self.water_current_direction_north)
        current_x = math.sin(theta)
        current_y = math.cos(theta)

        step_len = math.hypot(dx, dy)
        step_x = dx / step_len
        step_y = dy / step_len

        # +1 = moving with the current, -1 = moving straight into it.
        alignment = step_x * current_x + step_y * current_y

        strength = math.tanh(self.water_current_speed) * self.current_cost_weight
        multiplier = 1.0 - strength * alignment
        return min(max(multiplier, 0.2), 2.0)

    def manhattan_distance(self, node: GraphNode, goal_node: GraphNode):
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y)

    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
