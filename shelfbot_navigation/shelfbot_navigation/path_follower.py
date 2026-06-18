import rclpy
from rclpy.node import Node
import math
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


# Parameters
k = 0.1        # look forward gain
Lfc = 4      # base lookahead distance
WB = 1.0       # wheelbase in metres
TARGET_SPEED = 6.0  # m/s


class PurePursuitMotionPlanner(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        # Path subscriber to get the planned path from the A* planner
        self.path_subscriber = self.create_subscription(
            Path, 
            'planned_path', 
            self.path_callback, 
            10
            )
        
        # Localization subscriber to get the robot's current position and orientation
        self.localization_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, 
            'localization_result', 
            self.localization_callback, 
            10
            )

        # Publisher to send velocity commands to the robot
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
            )

        self.cx = []
        self.cy = []
        self.current_position = None
        self.current_yaw = None
        self.current_speed = 0.0
        self.target_ind = None
        self.old_nearest_index = None

        self.timer = self.create_timer(0.1, self.control_loop)

    # -- Callback functions for subscribers --
    def path_callback(self, msg):
        self.cx = [pose.pose.position.x for pose in msg.poses]
        self.cy = [pose.pose.position.y for pose in msg.poses]
        self.old_nearest_index = None
        self.target_ind = None
        self.get_logger().info(f'Got path with {len(self.cx)} waypoints')

    def localization_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    # -- Control loop for path following --
    
    def calc_distance(self, px, py):
        dx = self.current_position[0] - px
        dy = self.current_position[1] - py
        return math.hypot(dx, dy)

    def search_target_index(self):
        x, y = self.current_position

        if self.old_nearest_index is None:
            dx = [x - icx for icx in self.cx]
            dy = [y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = int(np.argmin(d))
            self.old_nearest_index = ind
        else:
            ind = self.old_nearest_index
            dist_this = self.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                if ind + 1 >= len(self.cx):
                    break
                dist_next = self.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if dist_this < dist_next:
                    break
                ind += 1
                dist_this = dist_next
            self.old_nearest_index = ind

        Lf = k * self.current_speed + Lfc

        while Lf > self.calc_distance(self.cx[ind], self.cy[ind]):
            if ind + 1 >= len(self.cx):
                break
            ind += 1

        return ind, Lf

    def control_loop(self):

        if not self.cx or self.current_position is None or self.current_yaw is None:
            return

        # Check if goal reached
        dx = self.cx[-1] - self.current_position[0]
        dy = self.cy[-1] - self.current_position[1]
        if math.hypot(dx, dy) < 0.3:
            self.stop()
            self.get_logger().info('Goal reached!')
            self.cx = []
            self.cy = []
            return

        target_ind, Lf = self.search_target_index()

        tx = self.cx[target_ind]
        ty = self.cy[target_ind]

        # Alpha is angle between heading and lookahead point
        alpha = math.atan2(ty - self.current_position[1], tx - self.current_position[0]) - self.current_yaw

        # Normalize
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi

        # Pure pursuit steering
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

        # Speed control
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