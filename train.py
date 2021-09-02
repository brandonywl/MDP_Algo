# import tensorflow as tf
import numpy as np
import pygame

from Env import Env
from reinforcement_learning.dq_agent import QNNAgent

# if __name__ == '__main__':
env = Env(display=True)
# curr_state = np.array(env.observation_space.ravel())

agent = QNNAgent(env)

# while not env.done:
#     action = agent.get_action(curr_state.reshape(-1,24))
#     next_state, reward, = env.update_step(action)
#     next_state = next_state.ravel()


repetitions = 0
train = True
reward = 0

total_rewards = []
repetition = 10
repetitions += repetition
episodes = 100
name = f"DQN-Agent-{repetitions}x{episodes}"
if train:
    # Results tend to even out at around 10 repetitions
    for rep in range(repetition):
        total_reward = 0
        for ep in range(episodes):
            observations = env.reset()
            done = False
            steps = 0
            while not done:
                steps += 1
                action = agent.get_action(observations.reshape((-1,24)))
                next_observations, reward = env.update_step(action)

                if steps > 10000:
                    env.done = True
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                        break
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            env.done = True

                done = env.done
                agent.train(observations.reshape((-1,24)), next_observations.reshape((-1,24)), action, reward, done)
                observations = next_observations
                print(f"Step: {steps}\t Repetition: {rep}\t Total Rewards: {total_rewards}\nEpisode: {ep}\t Total Reward: {total_reward}\t Action: {action}\t eps: {agent.eps}")
                # env.render()
                # clear_output(wait = True)
            total_reward += reward
        total_rewards.append(total_reward)

agent.model.save(".\\dqn_agent.h5")
    # end_training(dqagent, total_rewards, name)
# Load the previous model and results
# else:
#     train = load_model(dqagent, name)
#     train = True
#     if train:
#         print("Train set to true")
