class Agent():
  def __init__(self, env):
    ## Main
    self.action_discrete = True
    self.action_size = len(env.action_space)
    self.observation_shape = env.observation_shape
    # self.action_discrete = type(env.action_space) == gym.spaces.discrete.Discrete
    # self.observation_discrete = type(env.observation_space) == gym.spaces.discrete.Discrete
    #
    # if self.action_discrete:
    #   self.action_size = env.action_space.n
    #   print(f"Action Size: {self.action_size}\tDiscrete: {self.action_discrete}")
    # else:
    #   self.action_high = env.action_space.high
    #   self.action_low = env.action_space.low
    #   self.action_shape = env.action_space.shape
    #   print(f"Action High: {self.action_high}\t Action Low: {self.action_low}")
    #   print(f"Shape: {self.action_shape}\t Discrete: {self.action_discrete}")
    #
    # if self.observation_discrete:
    #   self.observation_size = env.observation_space.n
    #   print(f"Observation Size: {self.observation_size}\tDiscrete: {self.observation_discrete}")
    # else:
    #   self.observation_high = env.observation_space.high
    #   self.observation_low = env.observation_space.low
    #   self.observation_shape = env.observation_space.shape
    #   print(f"Observation High: {self.observation_high}\t Observation Low:{self.observation_low}")
    #   print(f"Shape: {self.observation_shape}\t Discrete: {self.observation_discrete}")