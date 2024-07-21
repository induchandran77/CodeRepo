import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models
import matplotlib.pyplot as plt

class MultiHopEnvironment:
    def __init__(self, num_uavs):
        self.num_uavs = num_uavs
        self.uavs = [{
            'computation_capacity': np.random.choice([500, 1000, 1500, 2000, 2500]),
            'initial_energy': 300000,
            'energy_level': 300000,  # in joules
            'task_size': np.random.randint(1000, 5000),  # Initial task size in bits
            'task_remaining': 0  # Initialize remaining task to offload
        } for _ in range(num_uavs)]
        self.init_environment_params()

    def init_environment_params(self):
        self.tx_power = [0.1] * self.num_uavs
        self.energy_per_cycle = 1e-9
        self.cpu_cycles_per_bit = 1000
        self.f = 2.4e9
        self.v = 3e8
        self.beta_LoS = 2
        self.P_noise = 10 ** (-13)
        self.B = 10 ** 6
        self.K = 10 ** (5 / 10)
        self.latencies = [[] for _ in range(self.num_uavs)]

    def calculate_channel_gain(self, distance):
        f = self.f
        v = self.v
        μ_LoS = 1.5  # Path loss factor
        β_LoS = self.beta_LoS
        return ((4 * np.pi * f * distance) / v)**(-β_LoS) * μ_LoS

    def step(self, actions):
        dones = [False] * self.num_uavs
        total_rewards = [0] * self.num_uavs
        total_energy_consumption = 0
        total_latency = 0

        for i in range(self.num_uavs):
            uav = self.uavs[i]
            action = actions[i]  # Fraction of the task to process locally
            process_amount = action * uav['task_size'] 
            offload_amount = uav['task_size'] - process_amount

            # Calculate energy and latency for computing
            energy_computing = self.cpu_cycles_per_bit * process_amount * self.energy_per_cycle
            latency_computing = self.cpu_cycles_per_bit * process_amount / uav['computation_capacity']

            # Calculate channel gain for offloading
            distance = 100  # Assume a default distance of 100 meters between UAVs
            channel_gain = self.calculate_channel_gain(distance)

            # Calculate energy and latency for offloading
            energy_offloading = offload_amount * self.tx_power[i] / (self.B * np.log2(1 + (self.tx_power[i] / (self.P_noise * channel_gain))))
            latency_offloading = offload_amount / (self.B * np.log2(1 + (self.tx_power[i] / (self.P_noise * channel_gain))))

            # Update energy level and task details
            total_energy_consumption = energy_computing + energy_offloading
            total_latency = max(latency_computing, latency_offloading)  # Considering the maximum as the effective latency

            uav['energy_level'] -= total_energy_consumption
            uav['energy_level'] = max(0, uav['energy_level'])  # Ensure energy doesn't go negative
            uav['task_size'] -= process_amount
            uav['task_remaining'] = offload_amount

            # Store latency
            self.latencies[i].append(total_latency)

            # Check if task is completed
            dones[i] = (uav['task_size'] + uav['task_remaining']) == 0
            total_rewards[i] -= (total_energy_consumption + total_latency) * 10  

        return self.get_state(), total_rewards, dones, {}

    def get_state(self):
        return [[uav['task_size'], uav['energy_level'], uav['computation_capacity'], uav['task_remaining']] for uav in self.uavs]
    
    def reset(self):
        # Reset each UAV's state to start a new episode
        for uav in self.uavs:
            uav['computation_capacity'] = np.random.choice([500, 1000, 1500, 2000, 2500])
            uav['initial_energy'] = 300000
            uav['energy_level'] = 300000  # Reset energy level to initial energy
            uav['task_size'] = np.random.randint(1000, 5000)  # Assign new initial task size
            uav['task_remaining'] = 0  # Reset any remaining task
            # Reset any additional state parameters as needed

        # Optionally reset other environment-specific metrics or logs, such as latencies
        self.latencies = [[] for _ in range(self.num_uavs)]

        return self.get_state()  # Return the initial state of the environment after resetting



class Agent:
    def __init__(self, state_dim, num_agents, agent_id, learning_rate=0.001):
        self.state_dim = state_dim
        self.num_agents = num_agents
        self.agent_id = agent_id
        self.learning_rate = learning_rate
        self.gamma = 0.995  # discount factor for future rewards
        self.mu = 0  # Initial value of the utility or performance metric
        self.previous_mu = 0
        self.Ct = np.ones((self.num_agents, self.num_agents)) / self.num_agents
        
        # Define the number of discrete actions (0.0 to 1.0, increment by 0.1)
        self.action_dim = 11

        # Neural networks for the actor and the critic
        self.actor = self.build_actor()
        self.critic = self.build_critic()
        
        # Optimizers
        self.actor_optimizer = tf.keras.optimizers.Adam(learning_rate)
        self.critic_optimizer = tf.keras.optimizers.Adam(learning_rate)

    # def update_mu(self, new_mu):
    #     self.previous_mu = self.mu
    #     self.mu = new_mu

    def update_consensus(self, Ct):
        self.Ct = Ct

    def build_actor(self):
        # Actor model to propose actions
        model = models.Sequential([
            layers.Dense(64, activation='relu', input_dim=self.state_dim, kernel_initializer='he_normal'),
            layers.Dense(self.action_dim, activation='softmax')
        ])
        return model

    def build_critic(self):
        # Critic model to evaluate the action
        model = models.Sequential([
            layers.Dense(64, activation='relu', input_dim=self.state_dim),
            layers.Dense(1)  # Value estimation for the state
        ])
        return model

    def act(self, state):
        #print("State:", state)
        # Reshape the state to be suitable for the neural network
        state = np.reshape(state, [1, self.state_dim])
        # Get action probabilities from the actor model
        logits = self.actor(state, training=False)
        # print("Logits:", logits)
        # epsilon = 1e-6
        # logits += epsilon

        action_probabilities = tf.nn.softmax(logits).numpy().flatten()

        # Check for NaN values in the probabilities
        if np.isnan(action_probabilities).any():
            print("NaN detected in action probabilities:", action_probabilities)
            action_probabilities = np.nan_to_num(action_probabilities, nan=1.0 / self.action_dim)
            action_probabilities /= action_probabilities.sum()  # Re-normalize to ensure they sum to 1

        # Choose an action based on the probabilities
        action_index = np.random.choice(self.action_dim, p=action_probabilities)
        return action_index / 10.0

    def decide_action(self, state, epsilon=0.1):
        # Random exploration vs exploitation decision
        if np.random.rand() < epsilon:
            # Exploration: random action from the discrete set
            return np.random.choice(self.action_dim) / 10.0  # Assuming action_dim is 11 for 0.0 to 1.0 in 0.1 increments
        else:
            # Exploitation: use the actor model to determine the best action
            state = np.reshape(state, [1, self.state_dim])
            action_probs = self.actor.predict(state)[0]
            action_index = np.argmax(action_probs)  # Select action with highest probability
            return action_index



    def train(self, state, action_index, reward, next_state, done):
        consensus_factor = np.sum(self.Ct[self.agent_id])  # Sum of weights for this agent
        adjusted_learning_rate = self.learning_rate * consensus_factor
        self.actor_optimizer.learning_rate = adjusted_learning_rate
        self.critic_optimizer.learning_rate = adjusted_learning_rate

        # Convert fraction back to action index for training
        action = action_index * 10

        state = np.reshape(state, [1, self.state_dim])
        next_state = np.reshape(next_state, [1, self.state_dim])
        
        with tf.GradientTape(persistent=True) as tape:
            # Predict the value of the current and next state
            critic_value = self.critic(state)
            critic_value_next = self.critic(next_state)
            td_target = reward + self.gamma * critic_value_next * (1 - int(done))
            td_error = td_target - critic_value
            #print("td_error",td_error)
            # Compute logits
            logits = self.actor(state)
            max_logit = tf.reduce_max(logits)
            logits -= max_logit
            # Compute action probabilities
            #action_probabilities = tf.nn.softmax(logits)

            # Compute actor loss using softmax cross-entropy
            action_one_hot = tf.one_hot(int(action), self.action_dim)
            actor_loss = tf.nn.softmax_cross_entropy_with_logits(labels=action_one_hot, logits=logits) * td_error
            
            # Critic loss to minimize the td_error
            critic_loss = tf.square(td_error)

        # Compute gradients for actor and critic
        actor_grads = tape.gradient(actor_loss, self.actor.trainable_variables)
        critic_grads = tape.gradient(critic_loss, self.critic.trainable_variables)

        # Clip gradients
        actor_grads = [tf.clip_by_value(grad, -1.0, 1.0) for grad in actor_grads]
        critic_grads = [tf.clip_by_value(grad, -1.0, 1.0) for grad in critic_grads]
        
        # Apply gradients
        self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
        self.critic_optimizer.apply_gradients(zip(critic_grads, self.critic.trainable_variables))

        # Clean up the tape
        del tape

        self.previous_mu = self.mu
        self.mu = critic_value.numpy()[0, 0]


def calculate_weights(agents, alpha):
    num_agents = len(agents)
    Ct = np.zeros((num_agents, num_agents))
    
    # Calculate learning progress
    LP = np.array([a.mu - a.previous_mu for a in agents])
    
    # Calculate Q values based on learning progress
    for i in range(num_agents):
        for j in range(num_agents):
            Qij = agents[i].learning_rate * LP[j] + agents[j].learning_rate * LP[i]
            Ct[i, j] = alpha * Ct[i, j] + (1 - alpha) * Qij
            
    return Ct / Ct.sum(axis=1, keepdims=True)

def main():
    num_uavs = 5
    environment = MultiHopEnvironment(num_uavs)
    agents = [Agent(state_dim=4, num_agents=num_uavs, agent_id=i, learning_rate=0.01) for i in range(num_uavs)]
    max_steps_per_episode = 200
    num_episodes = 500
    alpha = 0.5
    episode_rewards = []

    for episode in range(num_episodes):
        states = environment.reset()
        dones = [False] * num_uavs
        total_rewards = [0] * num_uavs
        steps = 0
        while not all(dones) and steps < max_steps_per_episode:
            actions = []
            for i in range(num_uavs):
                # Decide the fraction of the task to process locally based on the current state
                action_fraction = agents[i].decide_action(states[i])
                actions.append(action_fraction)  # Collect actions for each UAV

            # Environment processes these actions and returns new states, rewards, and done statuses
            next_states, rewards, dones, _ = environment.step(actions)

            # Update agents based on the new states and rewards
            for i in range(num_uavs):
                if not dones[i]:  # Only train if the episode is not done
                    # Convert fraction action back to index for training
                    action_index = int(actions[i] * 10)  
                    agents[i].train(states[i], action_index, rewards[i], next_states[i], dones[i])
                total_rewards[i] += rewards[i]

            if all(dones):
                print("All agents completed their episodes.")
                break

            Ct = calculate_weights(agents, alpha)  # Update consensus weights
            
            for agent in agents:
                agent.update_consensus(Ct)  # Apply consensus update to each agent

            states = next_states  # Update states for the next iteration
            steps += 1
        episode_rewards.append(sum(total_rewards))
        print(f"Episode {episode + 1} completed. Total Rewards: {total_rewards}")

    plt.plot(range(1, num_episodes + 1), episode_rewards)
    plt.xlabel('Episode')
    plt.ylabel('Total Rewards')
    plt.title('Rewards vs Episodes')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
