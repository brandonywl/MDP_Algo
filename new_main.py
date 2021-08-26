import time

import numpy as np
import pygame

from bicycle_movement_model import move
from map_tiles import drawing
from map_tiles.block import Block
from map_tiles.drawing import WINDOW_WIDTH, WINDOW_HEIGHT, CAR_WIDTH, CAR_LENGTH, CUBE_WIDTH, CUBE_LENGTH, RED, BLUE, \
    CYAN
from map_tiles.robot import Robot
from utilities import cm_to_pixel

CUBE_COLOR = CYAN
# CAR_COLOR = (255, 0, 0)

NUM_BLOCKS = 4

COMMAND_FREQUENCY = 20
MOVE_SPEED = 10
MOVE_SPEED_PIXEL = cm_to_pixel(MOVE_SPEED)

MAX_STEERING_ANGLE = 45


clock = pygame.time.Clock()

pygame.init()
window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

# Generate blocks for the map
blocks = Block.generate_blocks((CUBE_WIDTH, CUBE_LENGTH), 4)
robot = Robot(np.radians(90), CAR_WIDTH/2, (WINDOW_HEIGHT - CAR_LENGTH/4), CAR_WIDTH, CAR_LENGTH)
block_pos = [x.get_pos()[1:] for x in blocks]
# robot_pos = robot.get_pos()

# obstacles = [pygame.draw.rect(window, CUBE_COLOR, x) for x in block_pos]
# car = pygame.draw.rect(window, RED, robot_pos[1:])

running = True

delta_steer = 0
delta_forward = 0



while running:

    window.fill(drawing.BLACK)
    obstacles = drawing.draw_obstacles(window, block_pos)
    car = drawing.draw_robot(robot, window)

    pygame.display.update()
    clock.tick(100)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                delta_steer = np.radians(30)
            elif event.key == pygame.K_RIGHT:
                delta_steer = -np.radians(30)
            elif event.key == pygame.K_UP:
                delta_forward = MOVE_SPEED_PIXEL
            elif event.key == pygame.K_DOWN:
                delta_forward = -MOVE_SPEED_PIXEL
            elif event.key == pygame.K_ESCAPE:
                robot.set(CAR_WIDTH/2, WINDOW_HEIGHT-CAR_LENGTH/4, np.radians(90),0)
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_RIGHT or event.key == pygame.K_LEFT:
                delta_steer = 0.0
            if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                delta_forward = 0.0

    curr_point = robot.get_point()
    next_point = move(curr_point, delta_steer, delta_forward, CAR_LENGTH, MAX_STEERING_ANGLE, COMMAND_FREQUENCY)
    # Test if next_point collides with any obstacle, else stop


    robot.set(next_point.x, next_point.y, next_point.theta, delta_steer)


pygame.quit()