import rclpy
from rclpy.node import Node
import math
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool

# Parameters
Lf = 2              # base lookahead distance
WB = 1.5            # wheelbase in metres
TARGET_SPEED = 3.0  # m/s


class PurePursuitMotionPlanner(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        # Subscribe to the path
        self.path_subscriber = self.create_subscription(
            Path, 
            'planned_path', 
            self.path_callback, 
            10
            )
        
        # Subscribe to recieve current pose
        self.localization_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, 
            'localization_result', 
            self.localization_callback, 
            10
            )

        # Publish to control the robot
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
            )
        
        # Publish robot goal status

        self.goal_status_publisher = self.create_publisher(
            Bool,
            'status/navigation',
            10
        )

        self.goal_reached = Bool()
        self.goal_reached.data = False

        self.cx = []
        self.cy = []
        self.current_position = None
        self.current_yaw = None
        self.current_speed = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.cx = [pose.pose.position.x for pose in msg.poses]
        self.cy = [pose.pose.position.y for pose in msg.poses]

        self.goal_reached.data = False
        
        self.get_logger().info(f'Got path with {len(self.cx)} waypoints')

    def localization_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def calc_distance(self, px, py):
        dx = self.current_position[0] - px
        dy = self.current_position[1] - py
        return math.hypot(dx, dy)

    def search_target_index(self):
        x, y = self.current_position

        dx = [x - icx for icx in self.cx]
        dy = [y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        ind = int(np.argmin(d))

        while self.calc_distance(self.cx[ind], self.cy[ind]) < Lf:
            if ind + 1 >= len(self.cx):
                break
            ind += 1

        if ind > 0:
            p1x, p1y = self.cx[ind-1], self.cy[ind-1]
            p2x, p2y = self.cx[ind], self.cy[ind]
            d1 = self.calc_distance(p1x, p1y)
            d2 = self.calc_distance(p2x, p2y)
            t = (Lf - d1) / (d2 - d1)
            tx = p1x + t * (p2x - p1x)
            ty = p1y + t * (p2y - p1y)

        else:
            tx = self.cx[ind]
            ty = self.cy[ind]

        return tx, ty

    def control_loop(self):
        if not self.cx or self.current_position is None or self.current_yaw is None:
            return

        dx = self.cx[-1] - self.current_position[0]
        dy = self.cy[-1] - self.current_position[1]

        if math.hypot(dx, dy) < 0.3:
            self.stop()
            self.get_logger().info('☑️ Goal reached!')
            self.goal_reached.data = True
            self.goal_status_publisher.publish(self.goal_reached)
            self.cx = []
            self.cy = []
            return

        tx, ty = self.search_target_index()

        alpha = math.atan2(ty - self.current_position[1], tx - self.current_position[0]) - self.current_yaw

        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi

        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

        self.current_speed = TARGET_SPEED

        twist = Twist()
        twist.linear.x = self.current_speed
        twist.angular.z = delta * self.current_speed / WB

        self.cmd_vel_publisher.publish(twist)

    def stop(self):
        self.current_speed = 0.0
        self.cmd_vel_publisher.publish(Twist())


def main():
    rclpy.init()
    node = PurePursuitMotionPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()