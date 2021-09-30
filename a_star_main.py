import time
import math
import pygame

from Env import Env
from map_tiles.block import Block
from movement_models.hybrid_astar import Hybrid_AStar
from utilities import drawing
from utilities import utilities
from utilities.drawing import CUBE_WIDTH, CUBE_LENGTH


def runAStar(env):
    # Get robot start pos, target_node start pos
    # robot_start_pos = tuple(env.robot.get_point().as_list(False))
    # target_node_pos = tuple(Block.get_target_point(env.blocks[0]).as_list(False))
    robot_start_pos = tuple(env.robot.get_point().as_list(False))

    count = 1

    final_state_list = []
    indiv_state_list = []
    for block in env.blocks:
        # robot_start_pos = tuple(env.robot.get_point().as_list(False))
        target_node_pos = tuple(Block.get_target_point(block).as_list(False))
        print("Block ", count)
        print("Start node:", robot_start_pos)
        print("End node:", target_node_pos)

        a_star = Hybrid_AStar(robot_start_pos, target_node_pos, env.blocks)
        output = a_star.run()
        if output is None:
            return env, None
        if env.planned_path is None:
            env.planned_path = output[0]
        else:
            env.planned_path.extend(output[0])

        for instruction in output[1]:
            indiv_state_list.append(instruction)

        final_state_list.append(indiv_state_list)
        indiv_state_list = []
        robot_start_pos = output[0][-1][1]
        count += 1

    print()
    print("States: ")
    print(final_state_list)

    print("Start node:", robot_start_pos)
    print("End node:", target_node_pos)

    return env, final_state_list

def convertToSTM(final_state_list, vehicle_length):
    totalHWinstructions = []
    for stateList in final_state_list:
        totalHWinstructions.append(utilities.discreteInstructionToHardware(stateList, vehicle_length))
    print("HW instructions:")
    print(totalHWinstructions)
    return totalHWinstructions

def timeMeasure(env, n):
    noSoln = 0
    total_time = 0
    start_time = 0
    end_time = 0
    for i in range(n):
        start_time = time.time()
        output = runAStar(env)

        if not output:
            noSoln += 1
            print("No solution found for this iteration")

        end_time = time.time() - start_time
        total_time += end_time
        print("Iteration " + str(i+1) + " done")

    return total_time/n, noSoln

if __name__ == "__main__":

    block_1 = (0, 10, 0)
    block_2 = (6, 15, 0)
    block_3 = (10, 8, 180)
    block_4 = (6, 5, 270)
    blocks_pos = (block_1, block_2, block_3, block_4)

    blocks = Block.create_blocks(blocks_pos, (CUBE_WIDTH, CUBE_LENGTH))
    block_pos = [x.get_pos()[1:] for x in blocks]
    block_target_pos = [tuple(Block.get_target_point(block).as_list(False)) for block in blocks]

    # Initialize environment
    env = Env(num_blocks=5, display=True)


    env.blocks = blocks
    env.update()


    # initialise info about robot
    # WHEEL_LENGTH is 20
    # CAR_LENGTH is 60
    fwheelbwheelDist = drawing.CAR_LENGTH
    # steeringAngle in degrees, is 30 degrees
    steeringAngle = math.radians(env.STEERING_ANGLE)
    #turningRadius is about 103.923 pixels, converted to cm is about 25.98cm
    turningRadius = fwheelbwheelDist / math.tan(steeringAngle)

    """
    iterations = 50
    aveTime, noSoln = timeMeasure(env, iterations)
    print("Average time taken for " + str(iterations) + " times: " + str(aveTime) + "s")
    print("Total no solutions :" + str(noSoln))
    """

    env, output = runAStar(env)
    HWInstructions = convertToSTM(output, fwheelbwheelDist)
    
    
    allStr = []
    STMStr = ''
    for IndivIns in HWInstructions:
        for HWIns in IndivIns:
            action = HWIns[0]
            value = HWIns[1]
            # move backwards
            if HWIns[1] < 0:
                value *= -1
                if action == 'W':
                    action = 'S'
                elif action == 'A':
                    action = 'Z'
                elif action == 'D':
                    action = 'C'
            STMStr += action + str(round(value)) + '|'
            #HWIns = (action, value)
        allStr.append(STMStr)
        STMStr = ''

    print("Final string: ")
    for s in allStr:
        print(s)


    #print("Final list: ")
    #print(HWInstructions)



    # Run a* on it and nav in a update_step
    # a_star = Hybrid_AStar(robot_start_pos, target_node_pos, env.blocks)
    # output = a_star.run()
    # moves = output[1]
    # for move in moves:
    #     env.update_step_base(move[1], move[0])

    # drawing.draw_path(env.window, output[0])
    # env.planned_path = output[0]

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

