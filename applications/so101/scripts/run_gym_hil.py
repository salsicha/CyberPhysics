#!/usr/bin/env python3
"""
This script demonstrates how to run the gym_hil environment configured for the SO-101 arm.

It initializes a ROS 2 node, creates the Gymnasium environment, and then runs a simple
loop that takes random actions to showcase the interaction with the arm.
"""

import gymnasium as gym
import rclpy
# It's assumed that the gym_hil package is installed and registers its environments.
try:
    import gym_hil
except ImportError:
    print("Could not import gym_hil. Please ensure it is installed in your environment.")
    print("You might need to run: pip install gym-hil")
    exit(1)


def main(args=None):
    rclpy.init(args=args)

    print("Creating SO-101 HIL environment...")

    # The environment ID 'gym_hil/SO101-v0' is an example.
    # This needs to match the ID registered by the gym_hil package for the SO-101 arm.
    env = gym.make('gym_hil/SO101-v0')

    print("Environment created. Starting random action loop for 1000 steps...")

    try:
        # Reset the environment to get the initial observation
        observation, info = env.reset(seed=42)

        for i in range(1000):
            # Take a random action from the action space
            action = env.action_space.sample()

            # Apply the action to the environment
            observation, reward, terminated, truncated, info = env.step(action)

            # If the episode is over, reset the environment
            if terminated or truncated:
                print(f"Episode finished after {i+1} steps. Resetting environment.")
                observation, info = env.reset()

    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt. Shutting down.")
    finally:
        # Clean up the environment and ROS 2
        env.close()
        rclpy.shutdown()
        print("Environment closed and ROS 2 shut down.")

if __name__ == '__main__':
    main()