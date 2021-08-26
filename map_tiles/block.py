import numpy as np

from map_tiles.cell import Cell


class Block(Cell):
    def __init__(self, theta, x, y, block_width, block_height):
        super().__init__(theta, x, y, block_width, block_height)
        self.image_id = None

    @staticmethod
    def get_block_orientation(x, y):
        output_space = [0, 1, 2, 3]
        if x < 3:
            output_space.remove(2)
        if x > 16:
            output_space.remove(3)
        if y < 3:
            output_space.remove(1)
        if y > 16:
            output_space.remove(0)
        return np.random.choice(output_space)

    @staticmethod
    def map_orientation_to_theta(orientation_id):
        # Follows up down left right
        map_orientation_to_theta = {
            0: 0,
            1: 180,
            2: 270,
            3: 90
        }
        return map_orientation_to_theta[orientation_id]

    @staticmethod
    def get_new_xy(map_shape=(20, 20)):
        x = np.random.randint(0, map_shape[0])
        y = np.random.randint(0, map_shape[1])
        return x, y

    @staticmethod
    def generate_blocks(block_shape, count=1, seed=None, map_shape=(20, 20)):
        np.random.seed(seed) if seed else None
        blocks = []
        for _ in range(count):
            x, y = Block.get_new_xy(map_shape=map_shape)
            orientation_id = Block.get_block_orientation(x, y)
            theta = Block.map_orientation_to_theta(orientation_id)
            blocks.append(Block(theta, x, y, block_shape[0], block_shape[1]))
        return blocks
