import heapq

class MapPoint:
    def __init__(self, x, y, parent=None):
        self.px = x 
        self.py = y 
        self.prev = parent 
        self.cost_g = 0 
        self.cost_h = 0
        self.score_total = 0 
    
    def __lt__(self, other):
        return self.score_total < other.score_total

def a_star(raw_grid, p_start, p_goal):
    _h = len(raw_grid)
    _w = len(raw_grid[0])
    
    sx, sy = p_start
    gx, gy = p_goal
    
    if sx < 0 or sx >= _w or sy < 0 or sy >= _h: return None
    if gx < 0 or gx >= _w or gy < 0 or gy >= _h: return None
    
    # Check if the starting point is a wall
    if raw_grid[sy][sx] == 1: return None

    # Initialize the node
    node_start = MapPoint(sx, sy)
    node_goal = MapPoint(gx, gy)
  
    _priority_q = []
    heapq.heappush(_priority_q, (0, node_start))
    
    _checked_set = set()
    _best_cost_map = {(sx, sy): 0}
    
    # 8 directional movements
    _dirs = [
        (0, -1, 1), (0, 1, 1),      
        (-1, 0, 1), (1, 0, 1),      
        (-1, -1, 1.4), (-1, 1, 1.4),
        (1, -1, 1.4), (1, 1, 1.4)
    ]

    while _priority_q:
        _curr_score, curr_pt = heapq.heappop(_priority_q)
        if (curr_pt.px, curr_pt.py) in _checked_set: 
            continue
        
        # Goal reached
        if curr_pt.px == node_goal.px and curr_pt.py == node_goal.py:
            _final_path = []
            tmp = curr_pt
            while tmp:
                _final_path.append((tmp.px, tmp.py))
                tmp = tmp.prev
            return _final_path[::-1] 
        
        _checked_set.add((curr_pt.px, curr_pt.py))
        
        for dx, dy, weight in _dirs:
            nx = curr_pt.px + dx
            ny = curr_pt.py + dy
            
            # Boundary check
            if not (0 <= nx < _w and 0 <= ny < _h): continue
            if raw_grid[ny][nx] == 1: continue
            
            calc_g = curr_pt.cost_g + weight
            
            pos_tuple = (nx, ny)
            if pos_tuple not in _best_cost_map or calc_g < _best_cost_map[pos_tuple]:
                _best_cost_map[pos_tuple] = calc_g
                
                neighbor = MapPoint(nx, ny, curr_pt)
                neighbor.cost_g = calc_g
                neighbor.cost_h = abs(nx - node_goal.px) + abs(ny - node_goal.py)
                neighbor.score_total = neighbor.cost_g + neighbor.cost_h
                
                heapq.heappush(_priority_q, (neighbor.score_total, neighbor))
                
    return None
