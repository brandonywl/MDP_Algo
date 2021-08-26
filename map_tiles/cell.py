from map_tiles.point import Point


class Cell:
    def __init__(self, theta, x, y, cell_width, cell_height):
        self.x = x * cell_width
        self.y = y * cell_height
        self.cell_width = cell_width
        self.cell_height = cell_height
        self.theta = theta

    def get_pos(self):
        return self.theta, self.x, self.y, self.cell_width, self.cell_height

    def get_point(self):
        return Point(self.x, self.y, self.theta)