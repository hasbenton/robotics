import heapq

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0
    
    # [Critical Optimization] Allows nodes to be compared with the < operator, needed by heapq
    def __lt__(self, other):
        return self.f < other.f

def a_star(grid, start, goal):
    height = len(grid)
    width = len(grid[0])
    
    if not (0 <= start[0] < width and 0 <= start[1] < height): return None
    if not (0 <= goal[0] < width and 0 <= goal[1] < height): return None
    if grid[start[1]][start[0]] == 1: return None

    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    
    # [Critical Optimization] Use a min-heap (Priority Queue) instead of list iteration
    # This compresses multi-second calculations into milliseconds
    open_list = []
    heapq.heappush(open_list, (0, start_node))
    
    closed_set = set()
    g_score = {(start[0], start[1]): 0}
    
    # 8 directional movements
    movements = [
        (0, -1, 1), (0, 1, 1), (-1, 0, 1), (1, 0, 1),
        (-1, -1, 1.4), (-1, 1, 1.4), (1, -1, 1.4), (1, 1, 1.4)
    ]

    while open_list:
        # Retrieves the minimum value in O(1), very fast
        current_f, current_node = heapq.heappop(open_list)
        
        if (current_node.x, current_node.y) in closed_set: continue
        
        # Goal reached
        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            path = []
            curr = current_node
            while curr:
                path.append((curr.x, curr.y))
                curr = curr.parent
            return path[::-1]
        
        closed_set.add((current_node.x, current_node.y))
        
        for dx, dy, cost in movements:
            nx, ny = current_node.x + dx, current_node.y + dy
            
            if not (0 <= nx < width and 0 <= ny < height): continue
            if grid[ny][nx] == 1: continue
            
            new_g = current_node.g + cost
            
            if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                g_score[(nx, ny)] = new_g
                neighbor = Node(nx, ny, current_node)
                neighbor.g = new_g
                neighbor.h = abs(nx - goal_node.x) + abs(ny - goal_node.y)
                neighbor.f = neighbor.g + neighbor.h
                heapq.heappush(open_list, (neighbor.f, neighbor))
                
    return None
