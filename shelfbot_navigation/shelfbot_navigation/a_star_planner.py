class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, goal):

    open_list = []
    closed_list = []

    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0

    goal_node = Node(None, goal)
    goal_node.g = goal_node.h = goal_node.f = 0

    open_list.append(start_node)

    while open_list:

        current_node = open_list[0]
        current_index = 0

        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        
        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == goal_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path
        
        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
            
            # get the child node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure if it is within the range of the bounding box
            if node_position[0] > len(maze) - 1 or node_position[0] < 0 or node_position[1] < 0 or node_position[1] > len(maze[len(maze)-1]) - 1:
                continue
            
            # Check if the node is not a map obstacle and is walkable
            if maze[node_position[0]][node_position[1]] != 0:
                continue
            
            new_node = Node(current_node, node_position)
            children.append(new_node)
        
        # Loop through children
        for child in children:

            if child in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = ((child.position[0] - goal_node.position[0]) ** 2) + ((child.position[1] - goal_node.position[1]) ** 2)
            child.f = child.g + child.h

            if any(child == node and child.g > node.g for node in open_list):
                continue

            open_list.append(child)

def main():

    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    goal = (7, 6)

    path = astar(maze, start, goal)
    print(path)


if __name__ == '__main__':
    main()