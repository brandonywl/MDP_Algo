import numpy as np

from reinforcement_learning.q_agent import QAgent


class QNNAgent(QAgent):
    def build_table(self):
        self.model = tf.keras.models.Sequential(
            [
                tf.keras.layers.Dense(self.observation_size, input_shape=[self.observation_size]),
                tf.keras.layers.Dense(32, activation='relu'),
                tf.keras.layers.Dense(self.action_size)
            ]
        )

        loss_fn = tf.losses.MeanSquaredError()

        self.model.compile(
            optimizer='adam',
            loss=loss_fn,
            metrics=['mse', 'mae']
        )
        self.model.summary()

    def train(self, observation, next_observation, action, reward, done):
        a = self.observation_size
        observation = self.one_hot(observation, a)
        next_observation = self.one_hot(next_observation, a)
        q_init = self.model.predict(observation)
        q_next = self.model.predict(next_observation) if not done else np.zeros((1, self.action_size))
        q_target = reward + np.max(q_next) * self.discount_rate
        q_init[0, action] = q_target
        self.model.fit(x=observation, y=q_init, verbose=0)
        if done:
            self.eps *= 0.99

    def get_action(self, state):
        # Given current state, predict the q values of the next actions
        state = self.one_hot(state, self.observation_size)
        q_state = self.model.predict(state)
        # Greedy action = argmax(q_state)
        action_greedy = np.argmax(q_state)
        action_random = np.randint(0, self.action_size)
        return action_random if np.random.uniform() < self.eps else action_greedy

    def one_hot(self, inpt, size, ax=0, dtype='float32'):
        return tf.keras.utils.to_categorical(inpt, num_classes=size, dtype=dtype)