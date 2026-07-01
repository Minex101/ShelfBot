import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Bool

class ForkLifter(Node):
    def __init__(self):
        super().__init__('fork_mover')

        # Publisher to control the forklift
        self.forklift_publisher = self.create_publisher(
            JointState,
            'lift_cmd',
            10
        )

        # Publisher to send confirmation to state machine if pose has been reached
        self.status_publisher = self.create_publisher(
            Bool,
            'status/forklift',
            10
        )

        # Subscriber to get the forklift position to move to
        self.position_subscriber = self.create_subscription(
            Int32,
            'fork_goal_pose',
            self.position_callback,
            10
        )

        self.fork_position = None
        self.enabled = False
        self.current_position = 0.0
        self.step_size = 0.02
        self.pose_reached = Bool()

        self.create_timer(0.1, self.fork_mover)

    def position_callback(self, msg):

        self.fork_position = float(msg.data)
        self.enabled = True
    
    def fork_mover(self):

        if not self.enabled or self.fork_position is None: 
            return 
        
        # stop condition 
        if abs(self.current_position - self.fork_position) < 0.01:
            self.pose_reached.data = True
            self.status_publisher.publish(self.pose_reached)
            self.pose_reached.data = False   # reset for next goal
            return
        
        step = min(self.step_size, abs(self.fork_position - self.current_position))
        if self.current_position < self.fork_position:
            self.current_position += step
        else:
            self.current_position -= step
        
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['lift_joint']
        cmd.position = [self.current_position]

        self.forklift_publisher.publish(cmd)

        self.get_logger().info(f"Lift position: {self.current_position}")

def main(args=None):
    rclpy.init(args=args)
    node = ForkLifter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
