
import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self):
        self.map = None
        self.goal = None
        self.start = None
        self.row_trajectory = []
        self.col_trajectory = []
        self.row_kalman_pred = []
        self.col_kalman_pred = []

    def set_map(self, map, start, goal):
        self.map = map
        self.start = start
        self.goal = goal

    def set_path(self, path):
        self.path = path

    def plot_map(self):
        # plot the map
        rows, cols = self.map.shape
        # plot start and goal
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray', origin='lower')
        plt.plot(self.start[1], self.start[0], color='green', marker='s', label='Start')
        plt.plot(self.goal[1], self.goal[0], color='blue', marker='s', label='Goal')
        plt.xticks(range(0, cols, 5))
        plt.yticks(range(0, rows, 5))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()  # match grid orientation to the array
        plt.title("Grid Map with Start and Goal")
        plt.legend()
        plt.show()

    def plot_path(self):
        # plot the path
        rows, cols = self.map.shape
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray', origin='lower')
        path_rows, path_cols = zip(*self.path)
        plt.scatter(path_cols, path_rows, color='red', marker='s', label='Path')
        plt.text(self.start[1], self.start[0], 'S', color='red', fontsize=26, fontweight='bold', ha='center', va='center', label='Start')
        plt.text(self.goal[1], self.goal[0], 'G', color='red', fontsize=26, fontweight='bold', ha='center', va='center', label='Goal')
        plt.yticks(range(0, rows, 5))
        plt.xticks(range(0, cols, 5))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()
        plt.title("Grid Map with Path")
        plt.legend()
        plt.show()

    def plot(self, row_trajectory, col_trajectory, row_pred, col_pred):
        # plot the trajectory and kalman prediction
        rows, cols = self.map.shape
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray', origin='lower')
        path_rows, path_cols = zip(*self.path)
        plt.scatter(path_cols, path_rows, color='red', marker='s', label='Path')
        plt.text(self.start[1], self.start[0], 'S', color='red', fontsize=26, fontweight='bold', ha='center', va='center', label='Start')
        plt.text(self.goal[1], self.goal[0], 'G', color='red', fontsize=26, fontweight='bold', ha='center', va='center', label='Goal')
        plt.plot(col_trajectory, row_trajectory, color='green', marker='o', label='Trajectory')
        plt.plot(col_pred, row_pred, color='blue', marker='o', label='Kalman Prediction')
        plt.xticks(range(0, cols, 5))
        plt.yticks(range(0, rows, 5))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()
        plt.title("Grid Map with Trajectory and Kalman Prediction")
        plt.legend()
        plt.show()

    def plot_trajectory(self, row_trajectory, col_trajectory):
        # plot the trajectory
        rows, cols = self.map.shape
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray', origin='lower')
        path_rows, path_cols = zip(*self.path)
        plt.scatter(path_cols, path_rows, color='red', marker='s', label='Path')
        plt.text(self.start[1], self.start[0], 'S', color='red', fontsize=26, fontweight='bold', ha='center', va='center', label='Start')
        plt.text(self.goal[1], self.goal[0], 'G', color='red', fontsize=26, fontweight='bold', ha='center', va='center', label='Goal')
        plt.plot(col_trajectory, row_trajectory, color='green', marker='o', label='Trajectory')
        plt.xticks(range(0, cols, 5))
        plt.yticks(range(0, rows, 5))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()
        plt.title("Grid Map with Trajectory")
        plt.legend()
        plt.show()

    def plot_prediction(self, row_pred, col_pred):
        # plot the kalman prediction
        rows, cols = self.map.shape
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray', origin='lower')
        path_rows, path_cols = zip(*self.path)
        plt.scatter(path_cols, path_rows, color='red', marker='s', label='Path')
        plt.text(self.start[1], self.start[0], 'S', color='red', fontsize=26, fontweight='bold', ha='center', va='center', label='Start')
        plt.text(self.goal[1], self.goal[0], 'G', color='red', fontsize=26, fontweight='bold', ha='center', va='center', label='Goal')
        plt.plot(col_pred, row_pred, color='blue', marker='o', label='Kalman Prediction')
        plt.xticks(range(0, cols, 5))
        plt.yticks(range(0, rows, 5))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()
        plt.title("Grid Map with Kalman Prediction")
        plt.legend()
        plt.show()

    """
    def plot_trajectory(self, row_trajectory, col_trajectory):
        # plot the trajectory on the map
        rows, cols = self.map.shape
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray', origin='lower')
        if self.path != []:
            # plot the path
            path_rows, path_cols = zip(*self.path)
            plt.scatter(path_cols, path_rows, color='blue', marker='s', label='Path')

        plt.plot(col_trajectory, row_trajectory, color='red', marker='o', label='Trajectory')
        
        plt.xticks(range(0, cols, 5))
        plt.yticks(range(0, rows, 5))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()
        plt.title("Grid Map with Trajectory")
        plt.legend()
        plt.show()
    """
        
    def plot_kalman_prediction(self, x_pred, y_pred, angle_pred):
        # plot the kalman prediction
        plt.plot(x_pred, y_pred, label='Kalman Prediction')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Kalman Prediction')
        plt.legend()
        plt.show()
    
    def plot_map_given(self, map, start, goal):
        # plot the map
        rows, cols = map.shape
        # plot start and goal
        plt.figure(figsize=(10, 10))
        plt.imshow(map, cmap='gray', origin='lower')
        plt.plot(start[1], start[0], color='green', marker='o', label='Start')
        plt.plot(goal[1], goal[0], color='blue', marker='o', label='Goal')
        plt.xticks(range(0, cols, 5))
        plt.yticks(range(0, rows, 5))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()
        plt.title("Grid Map with Start and Goal")
        plt.legend()
        plt.show()
