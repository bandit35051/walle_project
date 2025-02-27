# movement_rl_env.py
import gym
from gym import spaces
import numpy as np

class MovementEnv(gym.Env):
    def __init__(self):
        super(MovementEnv, self).__init__()
        # Example state: [battery, front_distance, rear_distance]
        self.observation_space = spaces.Box(low=0, high=100, shape=(3,), dtype=np.float32)
        # Example actions: 5 discrete adjustments (0 to 4)
        self.action_space = spaces.Discrete(5)
        self.state = np.array([100, 50, 50], dtype=np.float32)
    
    def step(self, action):
        # Convert action into a numeric adjustment (-4, -2, 0, +2, +4)
        adjustment = (action - 2) * 2
        # Apply adjustment to simulated sensor readings
        self.state[1] += adjustment   # simulate change in front distance
        self.state[2] -= adjustment   # simulate change in rear distance
        # Reward: try to keep both distances close to 50
        reward = -abs(self.state[1] - 50) - abs(self.state[2] - 50)
        done = False  # In a real scenario, you'd define a termination condition
        return self.state, reward, done, {}
    
    def reset(self):
        self.state = np.array([100, 50, 50], dtype=np.float32)
        return self.state

if __name__ == "__main__":
    # Example training code using Stable Baselines3
    import stable_baselines3 as sb3
    env = MovementEnv()
    model = sb3.PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=10000)
    model.save("movement_rl_model")
