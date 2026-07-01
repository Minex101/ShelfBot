import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped

class State(Node):
    IDLE = 0
    PLAN = 1
    LOWER_FORK = 2
    DOCK = 3
    LIFT = 4

class ForkliftMissionStateMachine(Node):
    def __init__(self):
        super().__init__('forklift_mission')

        self.state = State.IDLE
        self.goal = None

        # Send the goal command for path planning
        self.goal_pub = self.create_publisher(
                PoseStamped,
                'goal_pose',
                10
            )
        
        # Send the command to dock the fork in pallete
        self.docker_pub = self.create_publisher(
            Bool,
            'status/docker',
            10
        )
        
        # Send the command to lift to goal height
        self.forklift_pub = self.create_publisher(
            Int32,
            'fork_goal_pose',
            10
        )
        
        # Get confirmation if the robot reached the goal position before docking
        self.create_subscription(
            Bool,
            'status/navigation',
            self.nav_status_callback,
            10
        )

        # Get confirmation if the robot has successfully lowered the forks
        self.create_subscription(
            Bool,
            'status/forklift',
            self.lift_status_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.tick)

        # Robot goal to pick pallete
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = 0.27
        self.goal.pose.position.y = 1.92
        self.goal.pose.position.z = 0.0
    
    def nav_status_callback(self, msg):
        if msg.data:
            self.state = State.LOWER_FORK
    
    def lift_status_callback(self, msg):
        if msg.data:
            self.state = State.DOCK

    def tick(self):

        if self.state == State.IDLE:
            self.state = State.PLAN

        elif self.state == State.PLAN:
            # Trigger path plan and move the robot to the goal
            self.goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(self.goal)
            self.get_logger().info("Goal sent to the planner! Executing motion")

        elif self.state == State.LOWER_FORK:
            # Lower the fork height before docking
            self.liftheight_goal = Int32()
            self.liftheight_goal.data = - 1
            self.forklift_pub.publish(self.liftheight_goal)

        elif self.state == State.DOCK:
            # Trigger docking stage
            self.dock = Bool()
            self.dock.data = True
            self.docker_pub.publish(self.dock)

        
def main(args=None):
    rclpy.init(args=args)
    node = ForkliftMissionStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()