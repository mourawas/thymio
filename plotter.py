import matplotlib.pyplot as plt
import numpy as np
import math

class Plotter:
    def __init__(self):
        pass

    def plot_path(self, path):
        # plot the path
        x_path = [point[0] for point in path]
        y_path = [point[1] for point in path]
        plt.plot(x_path, y_path, label='Path')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Path')
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