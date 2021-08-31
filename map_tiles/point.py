import numpy as np


class Point:
    def __init__(self, x, y, theta=None):
        self.x = x
        self.y = y
        self.theta = theta

    def as_list(self):
        return [self.x, self.y] if self.theta is None else [self.x, self.y, self.theta]

    def get_euclidean_dist(self, next_point):
        if self.theta is None:
            return np.sqrt(np.square(self.x - next_point.x) + np.square(self.y - next_point.y))
        else:
            # print(self.theta, next_point.theta)
            return np.sqrt(np.square(self.x - next_point.x) + np.square(self.y - next_point.y) + np.square(self.theta - next_point.theta))

    def print(self):
        print("x:", self.x, "y:", self.y, "theta:", self.theta)