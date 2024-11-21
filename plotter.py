import matplotlib.pyplot as plt
import numpy as np
import math

class Plotter:
    def __init__(self, data):
        self.data = data
        self.path = None

    def plot_path(self, path):
        self.path = path
        # Plot the data