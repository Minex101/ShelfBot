import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from std_msgs.msg import Bool
import tf2_geometry_msgs


class ForkLiftDocker(Node):
    def __init__(self):
        super().__init__('fork_docker')

        # Subscribe to check if docker needs to start
        self.create_subscription(
            Bool,
            'status/docker',
            self.enabledocker_callback,
            10
        )

        # Subscriber to see the ArUco pose / Pallete pose
        self.create_subscription(
            PoseStamped,
            '/aruco_pose',
            self.pose_callback,
            10
        )

        # Publish to move the robot to dock
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.docker_enabled = False
        self.kp = 3.5

        self.dock_speed = 1.0 # m/s

    def enabledocker_callback(self, msg):
        if msg.data == True:
            self.docker_enabled = True

    def pose_callback(self, msg):
        if not self.docker_enabled:
            return

        try:
            p = self.tf_buffer.transform(
                msg,
                'base_link',
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}')
            return

        x = p.pose.position.x
        y = p.pose.position.y
        z = p.pose.position.z
        distance = math.sqrt(x * x + y * y)
        error = math.atan2(y, x)

        cmd = Twist()

        if distance < 0.86:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.docker_enabled = False
            return

        cmd.angular.z = self.kp * math.sin(error)

        cmd.linear.x = self.dock_speed

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"x={x:.2f}, y={y:.2f}, z={z:.2f}, d={distance:.2f}, err={math.degrees(error):.1f}, "
            f"v={cmd.linear.x:.2f}, w={cmd.angular.z:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForkLiftDocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()