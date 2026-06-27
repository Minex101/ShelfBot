import rclpy
from rclpy.node import Node
import math
import heapq

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Subscriber to get the robot's position
        self.localization_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'localization_result',
            self.localization_callback,
            10
        )

        # Subscriber to get the map data
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            map_qos
        )

        # Subscriber to get the position (goal) to navigate to
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # Publisher to publish the planned path
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)

        self.current_position = None
        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin = None
        self.goal_position = None

    # -- Callback functions for subscribers --

    def localization_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f'📍 Current Position: {self.current_position}')

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position

        self.map_data = []
        for row in range(self.map_height):
            row_data = []
            for col in range(self.map_width):
                cell = msg.data[row * self.map_width + col]
                row_data.append(0 if cell == 0 else 1)
            self.map_data.append(row_data)

    def goal_callback(self, msg):
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'🎯 Goal Position: {self.goal_position}')
        if self.current_position and self.map_data:
            self.publish_path()

    # -- Helper functions to convert between world coordinates and grid coordinates --

    def world_to_grid(self, wx, wy):
        col = int((wx - self.map_origin.x) / self.map_resolution)
        row = int((wy - self.map_origin.y) / self.map_resolution)
        return (row, col)

    def grid_to_world(self, r, c):
        wx = self.map_origin.x + (c + 0.5) * self.map_resolution
        wy = self.map_origin.y + (r + 0.5) * self.map_resolution
        return wx, wy

    def publish_path(self):
        start = self.world_to_grid(*self.current_position)
        goal = self.world_to_grid(*self.goal_position)
        self.get_logger().info(f'Planning from {start} to {goal}')
        path = self.AStarPathPlanner(self.map_data, start, goal)
        if path:
            ros_path = Path()
            ros_path.header.frame_id = 'map'
            ros_path.header.stamp = self.get_clock().now().to_msg()

            for (r, c) in path:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                x, y = self.grid_to_world(r, c)
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                ros_path.poses.append(pose)

            self.path_publisher.publish(ros_path)
        else:
            self.get_logger().warn('⚠️ No path found!')
    
    def heuristic(self, a, b):
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def AStarPathPlanner(self, grid, start, goal):
        ' A* pathfinding algorithm implementation. Returns a list of grid coordinates from start to goal, or None if no path is found.'
        open_list = []
        heapq.heappush(open_list, (0 + self.heuristic(start, goal), start)) # Store tuples of (f_score, position)

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            current_position = heapq.heappop(open_list)[1] # Get the position with the lowest f_score

            if current_position == goal: # Goal reached, reconstruct the path by backtracking
                path = []
                while current_position in came_from:
                    path.append(current_position)
                    current_position = came_from[current_position]
                path.append(start)
                return path[::-1]
            
            # Get neighbors (up, down, left, right)
            x, y = current_position
            neighbors = [ (x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)] ]

            for neighbor in neighbors:
                if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0: # Check if neighbor is within bounds and not an obstacle
                    tentative_g_score = g_score[current_position] + 1 # Assuming cost between adjacent nodes is 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current_position
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
        return None

def main():

    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()