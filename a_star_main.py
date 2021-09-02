from Env import Env
from map_tiles.block import Block
from movement_models.hybrid_astar import Hybrid_AStar

if __name__ == "__main__":
    # Initialize environment
    env = Env(num_blocks=1, display=True)
    # Get robot start pos, target_node start pos
    robot_start_pos = tuple(env.robot.get_point().as_list(False))
    target_node_pos = tuple(Block.get_target_point(env.blocks[0]).as_list(False))

    # Run a* on it and nav in a update_step
    a_star = Hybrid_AStar(robot_start_pos, target_node_pos)
    output = a_star.run()
