import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math
import tf_transformations  # Make sure this is installed

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.subscription = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.path = []
        self.current_index = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0 

        self.timer = self.create_timer(0.1, self.follow_path)
        self.goal_reached = False

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_index = 0
        self.goal_reached = False
        self.get_logger().info(f'Received path with {len(self.path)} waypoints.')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.robot_yaw = yaw

    def follow_path(self):
        if not self.path or self.goal_reached:
            return

        if self.current_index >= len(self.path):
            self.stop_robot()
            self.get_logger().info('Reached Goal!')
            self.goal_reached = True
            return

        goal = self.path[self.current_index].pose.position
        dx = goal.x - self.robot_x
        dy = goal.y - self.robot_y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)

        angle_diff = self.normalize_angle(angle_to_goal - self.robot_yaw)

        cmd = Twist()
        
        if distance > 0.1:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.5 * angle_diff
        else:
            self.get_logger().info(f'Reached waypoint {self.current_index}')
            self.current_index += 1

        # Clamp speeds
        cmd.linear.x = max(min(cmd.linear.x, 0.3), -0.3)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
