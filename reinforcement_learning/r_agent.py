# An agent that picks a decision by random processes
class RAgent(Agent):
  def get_action(self, state):
    if self.action_discrete:
      action = random.randint(0, self.action_size - 1)
      return action
    else:
      return np.random.uniform(self.action_low, self.action_high, self.action_shape)