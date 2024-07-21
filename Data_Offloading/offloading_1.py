import numpy as np
import tensorflow as tf
from collections import deque
import random
import matplotlib.pyplot as plt

class MultiHopEnvironment:
    def __init__(self, num_uavs, initial_data):
        self.num_uavs = num_uavs
        self.remaining_data = initial_data
        self.computation_capacities = [5, 20, 20, 20, 20]  # Example computation capacities for each UAV
        self.energy_levels = [100] * num_uavs  # Initial energy levels for each UAV
        self.action_space = self.calculate_action_space()
        self.state_size = 2 * num_uavs + 1
        self.initial_data = initial_data
    
    def reset(self):
        # Reset the environment to its initial state
        self.remaining_data = self.initial_data
        self.state = [self.remaining_data] + self.computation_capacities + self.energy_levels
        return self.state
    
    def calculate_action_space(self):
        # Calculate action space for each UAV as discrete fractions
        num_actions = 11  # For example, 10 discrete actions
        action_space = []
        for capacity in self.computation_capacities:
            actions_for_uav = [i / (num_actions - 1) for i in range(num_actions)]
            action_space.append(actions_for_uav)
        print (action_space)
        return action_space

    def step(self, actions):
        # Perform actions for each UAV
        self.perform_actions(actions)
        # Update state (e.g., remaining data, energy levels)
        self.update_state()
        # Calculate reward based on latency or other relevant factors
        reward = self.calculate_reward()
        # Check if episode is done (e.g., all data processed)
        done = self.is_episode_done()
        return self.state, reward, done, {}
        
    def perform_actions(self, actions):
        # Offload data based on actions for each UAV
        for i, action in enumerate(actions):
            if self.energy_levels[i] > 0:
                processed_data = self.remaining_data * action
                self.remaining_data -= processed_data
                energy_consumption = processed_data / self.computation_capacities[i] * 10  # Example energy consumption factor
                self.energy_levels[i] -= energy_consumption
                self.energy_levels[i] = max(0, self.energy_levels[i])

    def update_state(self):
        self.state = [self.remaining_data] + self.computation_capacities + self.energy_levels
        
    def calculate_latency(self, action, capacity):
        # Calculate latency for processing based on action and capacity
        if capacity == 0:
            return float('inf')  # Avoid division by zero
        return (self.remaining_data * action) / capacity
    
    def calculate_reward(self):
        # Reward based on remaining data and latency
        latencies = [self.calculate_latency(action, capacity) for action, capacity in zip(self.state[1:self.num_uavs+1], self.computation_capacities)]
        overall_latency = sum(latencies)
        return -overall_latency
    
    def is_episode_done(self):
        return self.remaining_data <= 0

class DQNAgent:
    def __init__(self, state_size, action_size, num_uavs, action_space):
        self.state_size = state_size
        self.action_size = action_size
        self.num_uavs = num_uavs
        self.action_space = action_space
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95    # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.model = self._build_model()

    def _build_model(self):
        total_actions = len(self.action_space[0]) * self.num_uavs  # 11 actions per UAV * 5 UAVs = 55 actions
        model = tf.keras.models.Sequential([
            tf.keras.layers.Dense(24, input_dim=self.state_size, activation='relu'),
            tf.keras.layers.Dense(24, activation='relu'),
            tf.keras.layers.Dense(total_actions, activation='linear')  # Correct number of output neurons
        ])
        model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(learning_rate=self.learning_rate))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state, action_space):
        if np.random.rand() <= self.epsilon:
            return [random.choice(action_space[i]) for i in range(self.num_uavs)]
        act_values = self.model.predict(state)[0]
        actions = []
        for i in range(self.num_uavs):
            start_index = i * len(action_space[i])
            end_index = start_index + len(action_space[i])
            action_values_slice = act_values[start_index:end_index]
            if action_values_slice.size > 0:
                chosen_action_index = np.argmax(action_values_slice)
                actions.append(action_space[i][chosen_action_index])
            else:
                print(f"Error: empty slice detected for UAV {i}, defaulting to 0.0")
                actions.append(0.0)  # Default action if slice is empty or issue with slicing
        return actions

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target = (reward + self.gamma * np.max(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)[0]
            for i, a in enumerate(action):
                target_f[i] = target
            self.model.fit(state, target_f.reshape(1, -1), epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

def main():
    # Parameters
    num_uavs = 5
    initial_data = 1000
    state_size = 2 * num_uavs + 1
    action_size = 10  # Number of discrete actions per UAV
    
    # Initialize environment and agent
    env = MultiHopEnvironment(num_uavs, initial_data)
    agent = DQNAgent(state_size, action_size, num_uavs, env.action_space)
    
    # Training parameters
    episodes = 500
    batch_size = 32
    rewards = []
    
    for episode in range(episodes):
        state = env.reset()
        state = np.reshape(state, [1, state_size])
        done = False
        total_reward = 0
        while not done:
            action = agent.act(state, env.action_space)
            print("Action", action)
            next_state, reward, done, _ = env.step(action)
            next_state = np.reshape(next_state, [1, state_size])
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            total_reward += reward
            if len(agent.memory) > batch_size:
                agent.replay(batch_size)
        rewards.append(total_reward)
        print("Episode:", episode + 1, ", Total Reward:", total_reward)
    return rewards

if __name__ == "__main__":
    episode_rewards = main()
    plt.plot(episode_rewards)
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.title('Reward vs Episode')
    plt.grid(True)
    plt.show()