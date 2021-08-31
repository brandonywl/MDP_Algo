# Agent that uses a Q-Lookup Table (Specific form of Q-Learning) to approximate it's value function
import numpy as np


class QAgent(RAgent):
    def __init__(self, env, eps=1.00, discount_rate=0.99, learning_rate=0.01):
        super().__init__(env)
        self.eps = eps
        self.discount_rate = discount_rate
        self.learning_rate = learning_rate
        self.plot = None
        self.total_rewards = None
        self.build_table()

    # This assumes a discrete action and observation state
    def build_table(self):
        self.q_table = 1e-4 * np.random.random([self.observation_size, self.action_size])

    def get_action(self, state):
        if np.random.uniform() < self.eps:
            return super().get_action(state)
        else:
            # Greedy Choice
            action_values = self.q_table[state]
            action = np.argmax(action_values)
            return action

    def train(self, experience):
        state, next_state, reward, action, done = experience
        q_next = self.q_table[next_state]
        q_next = np.zeros([self.action_size]) if done or state == next_state else q_next
        q_target = reward + self.discount_rate * np.max(q_next)

        # Loss Fn
        q_update = q_target - self.q_table[state, action]

        # Update the table with a learning rate
        self.q_table[state, action] += self.learning_rate * q_update

        if done:
            self.eps *= 0.99

    def set_for_train(self, power=0, operand=10, epsilon=None):
        if epsilon != None:
            self.eps = epsilon
        change = operand ** (-1 * power)
        self.learning_rate *= change