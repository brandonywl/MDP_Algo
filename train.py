# import tensorflow as tf
import numpy as np

from Env import Env

if __name__ == '__main__':
    env = Env(display=True)
    curr_state = env.observation_space

    while not env.done:
        
        next_state, reward, = env.update_step(action)