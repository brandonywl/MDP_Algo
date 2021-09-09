import time

import pygame

from Env import Env
from map_tiles.block import Block
from movement_models.hybrid_astar import Hybrid_AStar
from utilities import drawing

if __name__ == "__main__":
    # Initialize environment
    env = Env(num_blocks=1, display=True)
    # Get robot start pos, target_node start pos
    robot_start_pos = tuple(env.robot.get_point().as_list(False))
    target_node_pos = tuple(Block.get_target_point(env.blocks[0]).as_list(False))

    print("Start node:", robot_start_pos)
    print("End node:", target_node_pos)
    # Run a* on it and nav in a update_step
    a_star = Hybrid_AStar(robot_start_pos, target_node_pos)
    output = a_star.run()
    moves = output[1]
    # for move in moves:
    #     env.update_step_base(move[1], move[0])

    # drawing.draw_path(env.window, output[0])
    env.planned_path = output[0]

    print(env.done)
    curr_action = 0
    total_reward = 0
    left_press = right_press = up_press = down_press = 0

    while not env.done:
        if True:
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
