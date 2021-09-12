import time

import numpy as np
import pygame

from map_tiles.block import Block
from map_tiles.robot import Robot
from movement_models import collision_detection
from movement_models.bicycle_movement_model import move, backaxel_move
from utilities import drawing
from utilities.drawing import CUBE_WIDTH, CUBE_LENGTH, CAR_WIDTH, CAR_LENGTH, WINDOW_HEIGHT, WINDOW_WIDTH, \
    WINDOW_OFFSET_WIDTH, WINDOW_OFFSET_HEIGHT
from utilities.utilities import cm_to_pixel


class Env:
    def __init__(self, display=True, num_blocks=5):
        self.map_shape = (20, 20)
        self.window_shape = (600, 600)

        self.NUM_BLOCKS = num_blocks

        self.COMMAND_FREQUENCY = 1
        self.MOVE_SPEED = 10
        self.MOVE_SPEED_PIXEL = cm_to_pixel(self.MOVE_SPEED)

        self.STEERING_ANGLE = 30
        self.MAX_STEERING_ANGLE = 45

        self.has_collision = False

        self.action_space = [
            (0, 0),
            (-self.STEERING_ANGLE, self.MOVE_SPEED),
            (0, self.MOVE_SPEED),
            (self.STEERING_ANGLE, self.MOVE_SPEED),
            (-self.STEERING_ANGLE, -self.MOVE_SPEED),
            (0, -self.MOVE_SPEED),
            (self.STEERING_ANGLE, -self.MOVE_SPEED)
        ]

        # self.action_space = [
        #     (-self.STEERING_ANGLE, self.MOVE_SPEED),
        #     (0, self.MOVE_SPEED),
        #     (self.STEERING_ANGLE, self.MOVE_SPEED),
        #     # (-self.STEERING_ANGLE, -self.MOVE_SPEED),
        #     # (0, -self.MOVE_SPEED),
        #     # (self.STEERING_ANGLE, -self.MOVE_SPEED)
        # ]

        self.COLLISION_DETECTION_ON = True
        self.done = False
        self.step_count = 0

        self.blocks = self.robot = self.block_pos = None
        self.setup_map()
        self.observation_space = self.get_state()
        self.observation_shape = self.observation_space.shape

        self.planned_path = None

        if display:
            self.window = pygame.display.set_mode((WINDOW_WIDTH + 2 * WINDOW_OFFSET_WIDTH,
                                                   WINDOW_HEIGHT + 2 * WINDOW_OFFSET_HEIGHT))
            self.update()
        else:
            self.window = None

    def step(self, action_idx):
        action = self.action_space[action_idx]
        delta_steer = action[0]
        delta_forward = action[1]
        return self.step_base(np.radians(delta_steer), delta_forward)

    def step_base(self, delta_steer, delta_forward):
        """
        Provides a single timestep for the environment. Has a reward function component that can be used in
        reinforcement learning or to calculate euclidean distance as a utility function for classical search algorithms
        :param delta_steer: Degree of steering
        :param delta_forward: Speed of car in pixels/s
        :return: State of the environment in the next time step as well as reward/utility of this step
        """
        step_reward = 0
        self.step_count += 1
        set_point = True
        curr_point = self.robot.get_point()
        # next_point = move(curr_point, delta_steer, delta_forward, CAR_LENGTH,
        #                   self.MAX_STEERING_ANGLE, self.COMMAND_FREQUENCY)
        print(delta_forward)
        next_point = backaxel_move(curr_point, delta_forward, delta_steer, CAR_LENGTH)
        if self.COLLISION_DETECTION_ON:
            if collision_detection.has_collision(next_point, self.blocks):
                print("Cannot move!")
                set_point = False
                next_point = curr_point
                self.has_collision = True
        if set_point:
            self.robot.set(next_point.x, next_point.y, next_point.theta, delta_steer)

        # Detect if next_point can see any blocks that has not been detected. If so, reward 1 point

        # Detect if no more blocks are unidentified, provide a reward of 5 points. Can be proportional to time taken
        # num_identified = sum([block.get_state()[-1] for block in self.blocks])
        # if num_identified == self.NUM_BLOCKS:
        #     step_reward += 5
        #     print("env.done set: Finished")
        #     self.done = True

        state = self.get_state()
        self.observation_space = state
        # Return environment state + reward
        step_reward = self.get_reward()

        return state, step_reward

    def update(self):
        """
        If update on UI is required on step
        """
        # Update the UI based on stored data
        # Print new screen
        self.window.fill(drawing.BLACK)

        # Print new grid
        drawing.draw_grid(self.window)

        # Print all blocks
        obstacles = drawing.draw_obstacles(self.window, self.blocks)

        # Print all target nodes
        targets = [Block.get_target_point(block) for block in self.blocks]
        orientation_ids = [block.orientation_id for block in self.blocks]
        drawing.draw_targets(self.window, targets, orientation_ids)

        # Print robot
        car = drawing.draw_robot(self.robot, self.window)

        # Print planned path if present
        if self.planned_path is not None:
            drawing.draw_path(self.window, self.planned_path)

        pygame.display.update()

    def update_step(self, action_idx):
        """
        To be called by UI (main) for each timestep with keypress
        :return:
        """
        output = self.step(action_idx)
        self.update()
        return output

    def update_step_base(self, delta_steer, delta_forward):
        output = self.step_base(delta_steer, delta_forward)
        self.update()
        return output

    def setup_map(self):
        self.blocks = Block.generate_blocks((CUBE_WIDTH, CUBE_LENGTH), self.NUM_BLOCKS)
        self.robot = Robot(np.radians(90), CAR_WIDTH / 2 + WINDOW_OFFSET_WIDTH,
                           (WINDOW_HEIGHT - CAR_LENGTH / 4 + WINDOW_OFFSET_HEIGHT), CAR_WIDTH, CAR_LENGTH)
        self.block_pos = [x.get_pos()[1:] for x in self.blocks]

    def reset(self):
        self.setup_map()
        self.done = False
        self.step_count = 0
        return self.get_state()

    def reset_robot(self):

        self.robot.set(CAR_WIDTH / 2 + WINDOW_OFFSET_WIDTH,
                       WINDOW_HEIGHT - CAR_LENGTH / 4 + WINDOW_OFFSET_HEIGHT, np.radians(90), 0)

    def get_state(self):
        # theta (deg), x, y, state
        # theta (deg), x, y, 0
        curr_block_states = np.array([block.get_state() for block in self.blocks])
        robot_state = np.array(self.robot.get_state())
        state = np.append(curr_block_states, robot_state.reshape((1, 4)), axis=0)
        return state

    def get_reward(self):
        """
        Put reward function or utility function here.
        :return: Current timestep reward or utility
        """
        # Reinforcement Learning #
        curr_reward = 0
        # Reward it for going close to a target node
        idx, e_dist = self.robot.get_min_euclidean_distance(self.blocks)
        curr_reward += 1/e_dist*100
        if e_dist < 20 and abs(self.robot.theta - np.radians(self.blocks[idx].theta)) < np.radians(20):
            print("Block detected!")
            curr_reward += (1/e_dist)*1000
            self.blocks[idx].identified = 1

        # Finished environment
        # for block in self.blocks:
        #     block.print_block()
        if sum([block.get_state()[-1] for block in self.blocks]) == self.NUM_BLOCKS:
            curr_reward += (1/e_dist)*10000
            print("env.done set: Finished")
            self.done = True

        if self.has_collision:
            curr_reward *= 0.1
        self.has_collision = False

        return curr_reward

    def is_done(self):
        if self.has_collision:
            self.done = True


if __name__ == '__main__':
    input_by_key = True

    env = Env(True)
    curr_action = 0
    total_reward = 0
    left_press = right_press = up_press = down_press = 0

    while not env.done:
        if input_by_key:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("env.done set: Quit")
                    env.done = True
                    break
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_LEFT:
                        left_press = 1
                    elif event.key == pygame.K_RIGHT:
                        right_press = 1
                    elif event.key == pygame.K_UP:
                        up_press = 1
                    elif event.key == pygame.K_DOWN:
                        down_press = 1
                    elif event.key == pygame.K_ESCAPE:
                        env.reset_robot()
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_RIGHT:
                        right_press = 0
                    elif event.key == pygame.K_LEFT:
                        left_press = 0
                    if event.key == pygame.K_UP:
                        up_press = 0
                    elif event.key == pygame.K_DOWN:
                        down_press = 0
            # If steering = 0, either both or none pressed. If steering = 1, left press, steering = -1, right press
            steering_direction = left_press - right_press
            movement_direction = up_press - down_press

            if movement_direction == 0:
                curr_action = 0

            elif steering_direction == -1:
                curr_action = 1 if movement_direction == 1 else 4
            elif steering_direction == 1:
                curr_action = 3 if movement_direction == 1 else 6
            else:
                curr_action = 2 if movement_direction == 1 else 5
        next_state, reward = env.update_step(curr_action)
        curr_state = next_state
        total_reward += reward
        # print(env.done, total_reward)
