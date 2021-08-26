class Point:
    def __init__(self, x, y, theta=None):
        self.x = x
        self.y = y
        self.theta = theta

    def as_list(self):
        return [self.x, self.y] if self.theta is None else [self.x, self.y, self.theta]