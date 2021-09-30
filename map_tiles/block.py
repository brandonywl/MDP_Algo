import numpy as np

import utilities.utilities
from map_tiles.cell import Cell
from map_tiles.point import Point
from utilities import drawing
from utilities.drawing import WINDOW_OFFSET_WIDTH, WINDOW_OFFSET_HEIGHT, WINDOW_HEIGHT


class Block(Cell):
    def __init__(self, theta, x, y, orientation_id, block_width, block_height):
        super().__init__(theta, x, y, block_width, block_height)
        self.orientation_id = orientation_id
        self.image_id = None
        self.range = utilities.utilities.cm_to_pixel(30, WINDOW_HEIGHT)
        self.identified = 0

    def get_state(self):
        return self.theta, self.x, self.y, self.identified

    def print_block(self):
        print("Theta:", self.theta, "\tx:", self.x, "\ty:", self.y, "\tidentified:", self.identified )

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
        theta = curr_block.theta
        range = curr_block.range

        return Block.get_target_point_base(x, y, theta, range)

    @staticmethod
    def get_target_point_base(x, y, theta, view_range):
        theta = np.radians(theta)
        delta_x = int(view_range * np.cos(theta))
        delta_y = int(view_range * np.sin(theta))

        new_x = x + delta_x
        new_y = y - delta_y
        new_theta = np.radians(180) + theta

        if new_theta > np.radians(360):
            new_theta -= np.radians(360)

        new_theta += np.radians(90)

        if new_theta > np.radians(360):
            new_theta -= np.radians(360)

        return Point(new_x, new_y, new_theta)


    @staticmethod
    def offset_block(block):
        block.x += WINDOW_OFFSET_WIDTH
        block.y += WINDOW_OFFSET_HEIGHT

    @staticmethod
    def is_between(new_xy, block_xy, target_xy):
        x, y = new_xy[0], new_xy[1]
        block_x = block_xy[0]
        block_y = block_xy[1]
        target_point_x = target_xy[0]
        target_point_y = target_xy[1]
        if block_x != target_point_x:
            if block_x < target_point_x:
                if block_x < x <= target_point_x:
                    return True and block_y == y
            else:
                if target_point_x <= x < block_x:
                    return True and block_y == y
        else:
            if block_y < target_point_y:
                if block_y < y <= target_point_y:
                    return True and block_x == x
            else:
                if target_point_y <= y < block_y:
                    return True and block_x == x
        return False

    @staticmethod
    def has_blocked_targets(new_pos, blocks):
        if not blocks:
            return False
        x = new_pos[0] * blocks[0].cell_width
        y = new_pos[1] * blocks[0].cell_height
        theta = new_pos[2]
        view_range = blocks[0].range
        new_target_point = Block.get_target_point_base(x, y, theta, view_range)
        for block in blocks:
            block_xy = (block.x, block.y)
            target_point = Block.get_target_point(block)
            target_point_xy = (target_point.x, target_point.y)
            if Block.is_between((x, y), block_xy, target_point_xy):
                return True
            if Block.is_between(block_xy, (x, y), (new_target_point.x, new_target_point.y)):
                return True
        return False


    @staticmethod
    def generate_blocks(block_shape, count=1, seed=None, map_shape=(20, 20)):
        np.random.seed(seed) if seed else None
        blocks = []
        prev_loc = {}
        for _ in range(count):
            x, y = Block.get_new_xy(map_shape=map_shape)
            orientation_id = Block.get_block_orientation(x, y)
            theta = Block.map_orientation_to_theta(orientation_id)
            while Block.is_illegal_block_placement((x, y, theta), blocks, map_shape, prev_loc):
                x, y = Block.get_new_xy(map_shape=map_shape)
                orientation_id = Block.get_block_orientation(x, y)
                theta = Block.map_orientation_to_theta(orientation_id)
            if x in prev_loc:
                prev_loc[x].append(y)
            else:
                prev_loc[x] = [y]

            orientation_id = Block.get_block_orientation(x, y)
            theta = Block.map_orientation_to_theta(orientation_id)
            block = Block(theta, x, y, orientation_id, block_shape[0], block_shape[1])
            Block.offset_block(block)
            blocks.append(block)
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

    @staticmethod
    def is_illegal_block_placement(new_pos, blocks, map_shape, prev_loc):
        x = new_pos[0]
        y = new_pos[1]
        # print("New pos", new_pos)
        # for idx, block in enumerate(blocks):
        #     print("Block", idx, block.x, block.y, block.theta)
        # # Test if block is currently in use
        clashes = x in prev_loc and y in prev_loc[x]
        # print(clashes)
        # # Test if blocks any existing target node
        clashes = clashes or Block.has_blocked_targets(new_pos, blocks)
        # print(clashes)
        # # Test if target node exists outside of the boundaries
        clashes = clashes or Block.is_invalid_block_location(x, y, map_shape)
        # # Test if target node is blocked

        # print(clashes)
        # print()
        return clashes

    @staticmethod
    def create_blocks(blocks_pos, block_shape):
        blocks = []
        for block_pos in blocks_pos:
            x, y, theta = block_pos
            orientation_id = Block.map_theta_to_orientation(theta)
            block = Block(theta, x, y, orientation_id, block_shape[0], block_shape[1])
            Block.offset_block(block)
            blocks.append(block)
        return blocks

    @staticmethod
    def map_theta_to_orientation(theta):
        map_theta_to_orientation = {
            90: 0,
            270: 1,
            180: 2,
            0: 3
        }

        return map_theta_to_orientation[theta]


if __name__ == "__main__":
    # print(Block.is_invalid_block_location(0, 17))

    # blocks = Block.generate_blocks((10, 10), 4)
    # block = blocks[0]
    # print(block.x, block.y, block.theta)
    # target = Block.get_target_point(block)
    # print(target.x, target.y, target.theta)

    block = Block(180, 18, 16, 1, drawing.CUBE_WIDTH, drawing.CUBE_LENGTH)
    new_x, new_y, theta = 19, 16, 180

    print(Block.has_blocked_targets((new_x, new_y, theta), [block]))

    # block = Block(0, 0, 0, 3, drawing.CUBE_WIDTH, drawing.CUBE_LENGTH)
    # new_x, new_y, theta = 1, 0, 0
    #
    # print(Block.has_blocked_targets((new_x, new_y, theta), [block]))