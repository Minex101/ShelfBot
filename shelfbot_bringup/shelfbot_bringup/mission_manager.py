"""
Mission Manager — Forklift State Machine
States: IDLE → NAVIGATE_TO_PALLET → DOCK → LIFT → NAVIGATE_TO_DROP → LOWER → DONE
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
import math

# ── State definitions ─────────────────────────────────────────────────────────
IDLE               = 'IDLE'
NAVIGATE_TO_PALLET = 'NAVIGATE_TO_PALLET'
WAITING_FOR_PALLET = 'WAITING_FOR_PALLET'
DOCK               = 'DOCK'
WAITING_FOR_DOCK   = 'WAITING_FOR_DOCK'
LIFT               = 'LIFT'
WAITING_FOR_LIFT   = 'WAITING_FOR_LIFT'
NAVIGATE_TO_DROP   = 'NAVIGATE_TO_DROP'
WAITING_FOR_DROP   = 'WAITING_FOR_DROP'
LOWER              = 'LOWER'
WAITING_FOR_LOWER  = 'WAITING_FOR_LOWER'
DONE               = 'DONE'
FAILED             = 'FAILED'

# ── Mission config — change these to your real coordinates ───────────────────
PALLET_POSE = {
    'x': 5.0, 'y': 3.0, 'z': 0.0,
    'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0
}
DROP_POSE = {
    'x': 10.0, 'y': 8.0, 'z': 0.0,
    'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0
}

GOAL_TOLERANCE  = 0.5   # metres — how close counts as "arrived"
LIFT_HEIGHT     = 0.3   # metres — fork lift height
LOWER_HEIGHT    = 0.0   # metres — fork lower height
LIFT_JOINT_NAME = 'fork_joint'  # change to your actual joint name


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ── Publishers ────────────────────────────────────────────────────────
        self.goal_pub  = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.lift_pub  = self.create_publisher(JointState,  '/lift_cmd',  10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Odometry,     '/odom',         self.odom_cb,    10)
        self.create_subscription(Path,         '/planned_path', self.path_cb,    10)

        # ── State machine ─────────────────────────────────────────────────────
        self.state          = IDLE
        self.current_x      = 0.0
        self.current_y      = 0.0
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.path_received  = False
        self.lift_sent      = False
        self.lift_timer     = 0

        # ── Main loop timer (10 Hz) ───────────────────────────────────────────
        self.create_timer(0.1, self.run)

        self.get_logger().info('Mission Manager started — state: IDLE')
        self.get_logger().info('Publishing first goal in 3 seconds...')

        # Auto-start after 3 seconds
        self.create_timer(3.0, self.start_mission)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def path_cb(self, msg: Path):
        if len(msg.poses) > 0:
            self.path_received = True

    # ── Helpers ───────────────────────────────────────────────────────────────

    def distance_to_goal(self):
        return math.sqrt(
            (self.current_x - self.current_goal_x) ** 2 +
            (self.current_y - self.current_goal_y) ** 2
        )

    def publish_goal(self, pose_dict):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = pose_dict['x']
        msg.pose.position.y = pose_dict['y']
        msg.pose.position.z = pose_dict['z']
        msg.pose.orientation.x = pose_dict['qx']
        msg.pose.orientation.y = pose_dict['qy']
        msg.pose.orientation.z = pose_dict['qz']
        msg.pose.orientation.w = pose_dict['qw']
        self.goal_pub.publish(msg)
        self.current_goal_x = pose_dict['x']
        self.current_goal_y = pose_dict['y']
        self.path_received  = False

    def publish_lift(self, height: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = [LIFT_JOINT_NAME]
        msg.position     = [height]
        msg.velocity     = [0.0]
        msg.effort       = [0.0]
        self.lift_pub.publish(msg)

    # ── State machine entry ───────────────────────────────────────────────────

    def start_mission(self):
        if self.state == IDLE:
            self.transition(NAVIGATE_TO_PALLET)

    def transition(self, new_state):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):

        # ── NAVIGATE TO PALLET ────────────────────────────────────────────────
        if self.state == NAVIGATE_TO_PALLET:
            self.get_logger().info('Publishing pallet goal...')
            self.publish_goal(PALLET_POSE)
            self.transition(WAITING_FOR_PALLET)

        elif self.state == WAITING_FOR_PALLET:
            dist = self.distance_to_goal()
            self.get_logger().info(
                f'Navigating to pallet — distance: {dist:.2f}m', throttle_duration_sec=2.0)
            if dist < GOAL_TOLERANCE:
                self.get_logger().info('Reached pallet!')
                self.transition(DOCK)

        # ── DOCK ──────────────────────────────────────────────────────────────
        elif self.state == DOCK:
            # TODO: trigger your pallet docker node here
            # e.g. publish to /dock/start topic
            self.get_logger().info('Docking... (add your docker trigger here)')
            self.lift_timer = 0
            self.transition(WAITING_FOR_DOCK)

        elif self.state == WAITING_FOR_DOCK:
            # TODO: wait for docker completion topic
            # For now wait 3 seconds as placeholder
            self.lift_timer += 1
            if self.lift_timer >= 30:  # 30 * 0.1s = 3 seconds
                self.get_logger().info('Dock complete!')
                self.transition(LIFT)

        # ── LIFT ──────────────────────────────────────────────────────────────
        elif self.state == LIFT:
            self.get_logger().info(f'Lifting fork to {LIFT_HEIGHT}m...')
            self.publish_lift(LIFT_HEIGHT)
            self.lift_timer = 0
            self.transition(WAITING_FOR_LIFT)

        elif self.state == WAITING_FOR_LIFT:
            self.lift_timer += 1
            if self.lift_timer >= 20:  # 2 seconds
                self.get_logger().info('Lift complete!')
                self.transition(NAVIGATE_TO_DROP)

        # ── NAVIGATE TO DROP ──────────────────────────────────────────────────
        elif self.state == NAVIGATE_TO_DROP:
            self.get_logger().info('Publishing drop location goal...')
            self.publish_goal(DROP_POSE)
            self.transition(WAITING_FOR_DROP)

        elif self.state == WAITING_FOR_DROP:
            dist = self.distance_to_goal()
            self.get_logger().info(
                f'Navigating to drop — distance: {dist:.2f}m', throttle_duration_sec=2.0)
            if dist < GOAL_TOLERANCE:
                self.get_logger().info('Reached drop location!')
                self.transition(LOWER)

        # ── LOWER ─────────────────────────────────────────────────────────────
        elif self.state == LOWER:
            self.get_logger().info('Lowering fork...')
            self.publish_lift(LOWER_HEIGHT)
            self.lift_timer = 0
            self.transition(WAITING_FOR_LOWER)

        elif self.state == WAITING_FOR_LOWER:
            self.lift_timer += 1
            if self.lift_timer >= 20:  # 2 seconds
                self.get_logger().info('Mission complete!')
                self.transition(DONE)

        # ── DONE ──────────────────────────────────────────────────────────────
        elif self.state == DONE:
            self.get_logger().info('✅ Mission complete! Robot idle.', throttle_duration_sec=5.0)

        elif self.state == FAILED:
            self.get_logger().error('❌ Mission failed!', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()