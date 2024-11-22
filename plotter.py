import matplotlib.pyplot as plt
import numpy as np
import math

class Plotter:
    def __init__(self):
        pass

    def set_map(self, map, start, goal):
        self.map = map
        self.goal = goal
        self.start = start

    def plot_map(self):
        # plot the map
        rows, cols = self.map.shape
        # plot start and goal
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray', origin='lower')
        plt.plot(self.start[1], self.start[0], color='green', marker='o', label='Start')
        plt.plot(self.goal[1], self.goal[0], color='blue', marker='o', label='Goal')
        plt.xticks(range(cols))
        plt.yticks(range(rows))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()  # Match grid orientation to the array
        plt.title("Grid Map with Start and Goal")
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
        plt.xticks(range(cols))
        plt.yticks(range(rows))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()
        plt.title("Grid Map with Start and Goal")
        plt.legend()
        plt.show()

    def plot_path(self, path):
        # plot the map
        rows, cols = self.map.shape
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray', origin='lower')

        # plot the path
        path_rows, path_cols = zip(*path)
        plt.plot(path_cols, path_rows, color='red', marker='o', label='Path')
        
        plt.xticks(range(cols))
        plt.yticks(range(rows))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        plt.gca().invert_yaxis()  # Match grid orientation to the array
        plt.title("Grid Map with Path")
        plt.legend()
        plt.show()

    def plot_trajectory(self, x_trajectory, y_trajectory):
        # plot the trajectory
        plt.plot(x_trajectory, y_trajectory, label='Trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Trajectory')
        plt.legend()
        plt.show()

    def plot_kalman_prediction(self, x_pred, y_pred, angle_pred):
        # plot the kalman prediction
        plt.plot(x_pred, y_pred, label='Kalman Prediction')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Kalman Prediction')
        plt.legend()
        plt.show()