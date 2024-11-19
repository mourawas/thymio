import numpy as np
import mat

class GlobalPlanning:
    
    def __inite__(self):
        self.magnification = 5
        self.map = []
        self.start = []
        self.goal = []
        self.path = [] #Shortest path
        
    #here is the function for the A* algortihm (extension of the dijkstra algorithm it's
    #why it's called dijkstra).
    def ext_dijkstra(self, matrix, start, goal):
        #If current goal equal old goal we don't do the algorithm
        if self.goal == goal:
            return self.path
            
        # Initialize variables
        self.map = matrix
        self.start = start
        self.goal = goal
        n = 0  # Distance from start
        grid = np.full_like(self.map, -4)  # Initialize grid with -2 (unmarked)
        current = None  # Current cell in path
        # Make obstacles bigger
        obstacles = []
        for cell in self.map:
            if self.map[cell] == -1:
                obstacles.append(cell)
        for obstacle in obstacles:
            if self.map[obstacle[0]-1, obstacle[1]] == 0:
                self.map[obstacle[0]-1:obstacle[0]-self.magnification, obstacle[1]] = -1
            if self.map[obstacle[0]+1, obstacle[1]] == 0:
                self.map[obstacle[0]+1:obstacle[0]+self.magnification, obstacle[1]] = -1
            if self.map[obstacle[0], obstacle[1]-1] == 0:
                self.map[obstacle[0], obstacle[1]-1:obstacle[1]-self.magnification] = -1
            if self.map[obstacle[0], obstacle[1]+1] == 0:
                self.map[obstacle[0], obstacle[1]+1:obstacle[1]+self.magnification] = -1
                
        # Mark the start cell with n
        grid[self.start] = n
    
        # Initialize a list to keep track of the frontier cells
        frontier = [self.start]
    
        while grid[self.goal] == -4:
            n += 1
            new_frontier = []
            # For each cell in the current frontier
            for cell in frontier:
                # Get neighbors of the cell (up, down, left, right)
                neighbors = [
                    (cell[0]-1, cell[1]),  # Up
                    (cell[0]+1, cell[1]),  # Down
                    (cell[0], cell[1]-1),  # Left
                    (cell[0], cell[1]+1)   # Right
                ]
                for neighbor in neighbors:
                    # Check if neighbor is within bounds and 
                    if (0 <= neighbor[0] < grid.shape[0]) and (0 <= neighbor[1] < grid.shape[1]):# Check if neighbor is within bounds
                        if grid[neighbor] == -4: # Unmarked cell  
                            if Map[neighbor] != -1: # Not an obstacle
                                # Mark neighbor with n
                                grid[neighbor] = n
                                # Add neighbor to new frontier
                                new_frontier.append(neighbor)
                                operation_count += 1
            if not new_frontier:
                # No more cells to explore, and goal is unreachable
                return None
            frontier = new_frontier
    
        # Reconstruct the path
        current = self.goal
        self.path.append(current)
        while current != self.start:
            neighbors = [
                (current[0]-1, current[1]),  # Up
                (current[0]+1, current[1]),  # Down
                (current[0], current[1]-1),  # Left
                (current[0], current[1]+1)   # Right
            ]
            # Find the neighbor with the lowest mark (n value)
            min_n = grid[current]
            next_cell = None
            for neighbor in neighbors:
                if (0 <= neighbor[0] < grid.shape[0]) and (0 <= neighbor[1] < grid.shape[1]): 
                    if grid[neighbor] >=0 and grid[neighbor] < min_n:
                        #The two absolut value is for the extension of the dijkstra algorithm
                        min_n = grid[neighbor] + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
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
    
    #need to put this in the main code

    GlobalPlanning.__inite__()
    #search the start and the goal
    for cell in matrix:
        if matrix[cell] == -2:
            start = cell
            break
    for cell in matrix:
        if matrix[cell] == -3:
            goal = cell
            break
            
    GlobalPlanning.ext_djikstra(matrix, start, goal)
    
    if path == None:
        print("No path found. Thymio cannot going to the goal, please reconsider the environment.")
