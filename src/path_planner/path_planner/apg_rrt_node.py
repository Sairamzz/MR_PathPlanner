import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import random


class PlannerNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def distance(n1, n2):
    return np.hypot(n1.x - n2.x, n1.y - n2.y)


def get_random_node(space):
    return PlannerNode(random.uniform(0, space[0]), random.uniform(0, space[1]))


def nearest_node(node_list, rnd_node):
    return min(node_list, key=lambda node: distance(node, rnd_node))


def steer(from_node, to_node, max_dist):
    dist = distance(from_node, to_node)
    if dist > max_dist:
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        return PlannerNode(from_node.x + max_dist * np.cos(theta),
                           from_node.y + max_dist * np.sin(theta))
    return to_node


def is_collision_free(node, occupancy_grid, resolution, map_origin):
    grid_x = int((node.x - map_origin[0]) / resolution)
    grid_y = int((node.y - map_origin[1]) / resolution)

    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    index = grid_y * width + grid_x

    if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
        return False

    return occupancy_grid.data[index] < 50


def backtrack_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


def apg_rrt(start, goal, space, occupancy_grid, resolution, map_origin,
            max_iter=500, max_dist=0.5):
    root = PlannerNode(*start)
    goal_node = PlannerNode(*goal)
    nodes = [root]

    for _ in range(max_iter):
        rnd_node = get_random_node(space)
        nearest = nearest_node(nodes, rnd_node)
        new_node = steer(nearest, rnd_node, max_dist)

        if is_collision_free(new_node, occupancy_grid, resolution, map_origin):
            new_node.parent = nearest
            nodes.append(new_node)

            if distance(new_node, goal_node) <= max_dist:
                goal_node.parent = new_node
                nodes.append(goal_node)
                return backtrack_path(goal_node)

    return None


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('apg_rrt_path_planner')
        self.publisher = self.create_publisher(Path, '/planned_path', 10)
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.occupancy_grid = None
        self.resolution = None
        self.map_origin = None

        self.start = (0.0, 0.0)
        self.goal = (2.5, 2.5)
        self.space = (5.0, 5.0)
        self.has_planned = False

        self.timer = self.create_timer(1.0, self.plan_and_publish)

    def map_callback(self, msg):
        self.occupancy_grid = msg
        self.resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def plan_and_publish(self):
        if not self.occupancy_grid:
            self.get_logger().info("Waiting for map...") # run turtlebot cartographer for map
            return
        
        if self.has_planned:
            return 

        path_points = apg_rrt(self.start, self.goal, self.space,
                              self.occupancy_grid, self.resolution, self.map_origin)

        if path_points:

            path_msg = Path()
            path_msg.header = Header()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'

            for (x, y) in path_points:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.01
                path_msg.poses.append(pose)

            self.publisher.publish(path_msg)
            self.get_logger().info(f"Path with {len(path_msg.poses)} points.")
            self.has_planned = True
        else:
            self.get_logger().info("Path not found")


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
