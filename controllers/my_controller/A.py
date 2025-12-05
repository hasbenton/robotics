class Node:
    """Represents a node in the A* algorithm."""
    def __init__(self, x, y, parent=None):
        self.x = x          # Grid column (x-coordinate)
        self.y = y          # Grid row (y-coordinate)
        self.parent = parent# Parent node for path tracking
        self.g = 0          # Cost from start to this node
        self.h = 0          # Heuristic cost to goal
        self.f = 0          # Total cost (g + h)
    
    def __eq__(self, other):
        """Check if two nodes are the same (same coordinates)."""
        return self.x == other.x and self.y == other.y

def a_star(grid, start, goal):
    """
    Find shortest path from start to goal using A* algorithm.
    Args:
        grid: 2D list (0 = free, 1 = obstacle)
        start: (x, y) grid coordinates of start
        goal: (x, y) grid coordinates of goal
    Returns:
        Path as list of (x, y) coordinates, or None if no path exists
    """
    # Initialize start and goal nodes
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    
    # Open list (nodes to explore), closed list (nodes explored)
    open_list = [start_node]
    closed_list = []
    
    while open_list:
        # Get node with lowest f-cost
        current_node = open_list[0]
        current_idx = 0
        for idx, node in enumerate(open_list):
            if node.f < current_node.f:
                current_node = node
                current_idx = idx
        
        open_list.pop(current_idx)
        closed_list.append(current_node)
        
        # Check if goal is reached
        if current_node == goal_node:
            path = []
            current = current_node
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]  # Reverse to get start -> goal
        
        # Generate 4-directional neighbors (up, down, left, right)
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            x = current_node.x + dx
            y = current_node.y + dy
            
            # Check if neighbor is within grid bounds and not an obstacle
            if (0 <= x < len(grid[0])) and (0 <= y < len(grid)) and (grid[y][x] == 0):
                neighbors.append(Node(x, y, current_node))
        
        # Evaluate neighbors
        for neighbor in neighbors:
            # Skip if neighbor is already explored
            if neighbor in closed_list:
                continue
            
            # Calculate costs
            neighbor.g = current_node.g + 1  # Cost per step = 1
            # Manhattan heuristic (distance in grid cells)
            neighbor.h = abs(neighbor.x - goal_node.x) + abs(neighbor.y - goal_node.y)
            neighbor.f = neighbor.g + neighbor.h
            
            # Skip if neighbor is in open list with lower cost
            if any(neighbor == n and neighbor.g >= n.g for n in open_list):
                continue
            
            open_list.append(neighbor)
    
    # No path found
    return None

# Example Usage
if __name__ == "__main__":
    # Define start and goal in grid coordinates (matches test_maps.py)
    start = (1, 1)    # (x, y) in grid (inside map1's inner area)
    goal = (20, 20)   # (x, y) in grid (opposite corner)
    
    # Find path
    path = a_star(grid, start, goal)
    
    if path:
        print("Shortest path (grid coordinates):")
        for point in path:
            print(f"({point[0]}, {point[1]})")
    else:
        print("No path exists!")