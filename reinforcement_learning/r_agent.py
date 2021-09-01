# An agent that picks a decision by random processes
import numpy as np

from reinforcement_learning.agent import Agent


class RAgent(Agent):
    def get_action(self, state):
        if self.action_discrete:
            action = np.random.randint(0, self.action_size)
            return action
