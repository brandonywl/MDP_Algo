import numpy as np

import utilities.utilities
from map_tiles.cell import Cell
from map_tiles.point import Point
from utilities import drawing


class Block(Cell):
    def __init__(self, theta, x, y, orientation_id, block_width, block_height):
        super().__init__(theta, x, y, block_width, block_height)
        self.orientation_id = orientation_id
        self.image_id = None
        self.range = utilities.utilities.cm_to_pixel(20, drawing.WINDOW_HEIGHT)

    @staticmethod
    def get_block_orientation(x, y):
        output_space = [0, 1, 2, 3]
        if x < 3:
            output_space.remove(2)
        if x > 16:
            output_space.remove(3)
        if y < 3:
            output_space.remove(0)
        if y > 16:
            output_space.remove(1)
        return np.random.choice(output_space)

    @staticmethod
    def map_orientation_to_theta(orientation_id):
        # Follows up down left right
        map_orientation_to_theta = {
            0: 90,
            1: 270,
            2: 180,
            3: 0
        }
        return map_orientation_to_theta[orientation_id]

    @staticmethod
    def get_new_xy(map_shape=(20, 20)):
        x = np.random.randint(0, map_shape[0])
        y = np.random.randint(0, map_shape[1])
        return x, y

    @staticmethod
    def get_target_point(curr_block):
        x = curr_block.x
        y = curr_block.y
        theta = np.radians(curr_block.theta)
        delta_x = int(curr_block.range * np.cos(theta))
        delta_y = int(curr_block.range * np.sin(theta))

        # If facing rightward or downward, need to offset by 1 square
        new_x = x + delta_x
        new_y = y - delta_y

        new_theta = np.radians(180) + theta

        return Point(new_x, new_y, new_theta)

    @staticmethod
    def generate_blocks(block_shape, count=1, seed=None, map_shape=(20, 20)):
        np.random.seed(seed) if seed else None
        blocks = []
        prev_loc = {}
        for _ in range(count):
            x, y = Block.get_new_xy(map_shape=map_shape)

            # # Test if clashes with any existing blocks
            # (x in prev_loc and y in prev_loc[x]
            # # Test if blocks any existing target node
            # # Test if target node exists outside of the boundaries

            while Block.is_invalid_block_location(x, y, map_shape) or (x in prev_loc and y in prev_loc[x]):
                x, y = Block.get_new_xy(map_shape=map_shape)
            if x in prev_loc:
                prev_loc[x].append(y)
            else:
                prev_loc[x] = [y]

            orientation_id = Block.get_block_orientation(x, y)
            theta = Block.map_orientation_to_theta(orientation_id)
            blocks.append(Block(theta, x, y, orientation_id, block_shape[0], block_shape[1]))
        return blocks

    @staticmethod
    def is_invalid_block_location(x, y, map_shape=(20, 20)):
        """
        Follows convention where x,y of the block is the top left of the block
        :param x: x coordinate of the block
        :param y: y coordinate of the block
        :param map_shape: Shape in the map in number of cells
        :return: Boolean representing whether that x, y coordinate is invalid
        """
        # Test whether it is out of bounds of the map
        if x < 0 or x > map_shape[0]:
            return True
        if y < 0 or y > map_shape[1]:
            return True
        # Test whether it is within the range of the starting 2x2 cell, giving 1 extra cell for maneuvering out
        if x < 3 and y > 16:
            return True

        return False


if __name__ == "__main__":
    # print(Block.is_invalid_block_location(0, 17))

    blocks = Block.generate_blocks((10, 10), 4)
    block = blocks[0]
    print(block.x, block.y, block.theta)
    target = Block.get_target_point(block)
    print(target.x, target.y, target.theta)
