import numpy as np
import math

class GlobalPlanning:
    def __init__(self):
        self.magnification = 0
        self.map = None
        self.start = None
        self.goal = None
        self.path = [] #Shortest path
    
    def set_magnification(self, scale, thymio_size):
        # scale is cells/mm
        # I want the magnification in cells
        self.magnification = math.ceil(thymio_size * scale)
        print("GLOBAL PLANNING: magnification: ", self.magnification)
        
    #here is the function for the dijkstra algortihm
    def dijkstra(self, matrix, start, goal):
        self.path = []

        # Initialize variables
        self.map = matrix
        self.start = start
        self.goal = goal
        n = 0  # Distance from start
        grid = np.full_like(self.map, -4)  # Initialize grid with -4 (unmarked)
        current = None  # Current cell in path

        area = math.ceil(self.magnification * 1.5)
        shadow_area = []
        for drow in range(-area, area + 1):
            for dcol in range(-area, area + 1):
                shadow_cell = (start[0] + drow, start[1] + dcol)
                if (0 <= shadow_cell[0] < matrix.shape[0]) and (0 <= shadow_cell[1] < matrix.shape[1]):
                    shadow_area.append(shadow_cell)
        
        for cell in shadow_area:
            matrix[cell] = 0

        # Make obstacles bigger
        obstacles = []
        for row in range(grid.shape[0]):
            for col in range(grid.shape[1]):
                if self.map[row, col] == -1:
                    obstacles.append((row, col))
        for obstacle in obstacles:
            neighbors = []
            for drow in range(-self.magnification, self.magnification + 1):
                for dcol in range(-self.magnification, self.magnification + 1):
                    if drow != 0 or dcol != 0:  # Exclude the obstacle itself
                        neighbors.append((obstacle[0] + drow, obstacle[1] + dcol))
            for neighbor in neighbors:
                if (0 <= neighbor[0] < grid.shape[0]) and (0 <= neighbor[1] < grid.shape[1]):
                    self.map[neighbor] = -1
                
        # Mark the start cell with n
        grid[self.start] = n
    
        # Initialize a list to keep track of the frontier cells
        frontier = [self.start]
    
        while grid[self.goal] == -4:
            n += 1
            new_frontier = []
            # For each cell in the current frontier
            for cell in frontier:
                # Get neighbors of the cell (up, down, left, right and diagonals)
                neighbors = [
                    (cell[0]-1, cell[1]),   # Up
                    (cell[0]+1, cell[1]),   # Down
                    (cell[0], cell[1]-1),   # Left
                    (cell[0], cell[1]+1),   # Right
                    (cell[0]-1, cell[1]-1), # Up Left
                    (cell[0]-1, cell[1]+1), # Up Right
                    (cell[0]+1, cell[1]-1), # Down Left
                    (cell[0]+1, cell[1]+1)  # Down Right
                ]
                for neighbor in neighbors:
                    if (0 <= neighbor[0] < grid.shape[0]) and (0 <= neighbor[1] < grid.shape[1]):# Check if neighbor is within bounds
                        if grid[neighbor] == -4: # Unmarked cell  
                            if matrix[neighbor] != -1: # Not an obstacle
                                # Mark neighbor with n
                                grid[neighbor] = n
                                # Add neighbor to new frontier
                                new_frontier.append(neighbor)
            if not new_frontier:
                # No more cells to explore, and goal is unreachable
                return None
            frontier = new_frontier
    
        # Reconstruct the path
        current = self.goal
        self.path.append(current)
        while current != self.start:
            neighbors = [
                (current[0]-1, current[1]),     # Up
                (current[0]+1, current[1]),     # Down
                (current[0], current[1]-1),     # Left
                (current[0], current[1]+1),     # Right
                (current[0]-1, current[1]-1),   # Up Left
                (current[0]-1, current[1]+1),   # Up Right
                (current[0]+1, current[1]-1),   # Down Left
                (current[0]+1, current[1]+1)    # Down Right
            ]
            # Find the neighbor with the lowest mark (n value)
            min_n = grid[current]
            next_cell = None
            for neighbor in neighbors:
                if (0 <= neighbor[0] < grid.shape[0]) and (0 <= neighbor[1] < grid.shape[1]): 
                    if grid[neighbor] >= 0 and grid[neighbor] < min_n:
                        min_n = grid[neighbor]
                        next_cell = neighbor
                        
            if next_cell is None:
                # No path found
                return None
            # Append next_cell to path
            self.path.append(next_cell)
            current = next_cell
        # Reverse the path
        self.path.reverse()
        return self.path

    def get_matrix(self):
        return self.map