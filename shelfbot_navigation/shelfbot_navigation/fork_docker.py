import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class ForkLiftDocker(Node):
    def __init__(self):
        super().__init__('fork_docker')

        self.aruco_pose_subscriber = self.create_subscription(
            PoseStamped, 
            '/aruco_pose', 
            self.pose_callback, 
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
            )
        
        self.base_frame = 'base_link'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.standoff = 0.3
        self.Kp_alpha = 0.4
        self.Kp_yaw = 0.0
        self.Kp_v = 1.0
    
    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_callback(self, msg):
        self.last_pose_time = self.get_clock().now()

        pose_in_base = self.tf_buffer.transform(
            msg, 
            self.base_frame, 
            timeout=rclpy.duration.Duration(seconds=0.1)
        )

        x = pose_in_base.pose.position.x
        y = pose_in_base.pose.position.y

        self.alpha = math.atan2(y, -x) # Heading Angle Error
        self.yaw = self.quat_to_yaw(pose_in_base.pose.orientation) # pallet yaw error
        self.x_err = -x - self.standoff # distance between the pallete and robot

        if abs(self.yaw) > math.radians(5):  # phase 1: face the pallet head on
            omega = self.Kp_yaw * self.yaw
            v = 0.0
        elif abs(y) > 0.1:  # phase 2: lateral alignment
            omega = self.Kp_alpha * y
            v = 0.0
        else:  # phase 3: drive straight in
            omega = 0.0
            v = self.Kp_v * self.x_err
            
        v = self.Kp_v * self.x_err

        if self.x_err <= 0:
            v = 0.0

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"x_err={self.x_err:.3f} alpha_deg={math.degrees(self.alpha):.1f} "
            f"v={v:.3f} omega={omega:.3f} yaw={self.yaw:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ForkLiftDocker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()