#code from ddpg_pendulum.py example keras-rl 
#https://github.com/keras-rl/keras-rl/blob/master/examples/ddpg_pendulum.py

import numpy as np
import gym
import gymNaoEnv
import pickle
import logging

from datetime import datetime
import time

from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess


#Logging from: https://github.com/olavt/gym_co2_ventilation/blob/master/examples/test_keras_rl_continious.py
logger = logging.getLogger("Logger")
ch = logging.StreamHandler()
formatter = logging.Formatter(fmt='%(asctime)s.%(msecs)03d - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
ch.setFormatter(formatter)
logger.addHandler(ch)
logger.setLevel(logging.ERROR)

# Initialize logger for logging summary for each episode in the continuous learning process
episode_logger = logging.getLogger("EpisodeLogger")
episode_logger.setLevel(logging.INFO)
fh = logging.FileHandler("nao_learning_log_{}.log".format(datetime.now().strftime("%Y_%m_%d_%H%M")), mode="w")
episode_logger.addHandler(fh)
episode_logger.info("Time,Episode,TrainReward,TestReward")
formatter = logging.Formatter(fmt='%(asctime)s.%(msecs)03d,%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
fh.setFormatter(formatter)

ENV_NAME = 'basic-v0'

env = gym.make(ENV_NAME)
np.random.seed(123)
env.seed(123)

nb_episodes = 1

assert len(env.action_space.shape) == 1
nb_actions = env.action_space.shape[0]

# Next, we build a very simple model.
actor = Sequential()
actor.add(Flatten(input_shape=(1,) + env.observation_space.shape))
actor.add(Dense(16))
actor.add(Activation('relu'))
actor.add(Dense(16))
actor.add(Activation('relu'))
actor.add(Dense(16))
actor.add(Activation('relu'))
actor.add(Dense(nb_actions))
actor.add(Activation('linear'))
print(actor.summary())

action_input = Input(shape=(nb_actions,), name='action_input')
observation_input = Input(shape=(1,) + env.observation_space.shape, name='observation_input')
flattened_observation = Flatten()(observation_input)
x = Concatenate()([action_input, flattened_observation])
x = Dense(32)(x)
x = Activation('relu')(x)
x = Dense(32)(x)
x = Activation('relu')(x)
x = Dense(32)(x)
x = Activation('relu')(x)
x = Dense(1)(x)
x = Activation('linear')(x)
critic = Model(inputs=[action_input, observation_input], outputs=x)
print(critic.summary())

# Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
# even the metrics!
try:
    memory = pickle.load(open("memory.pkl", "rb"))
except (IOError, EOFError):
    memory = SequentialMemory(limit=100000, window_length=1)


random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.3)
agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
                  memory=memory, nb_steps_warmup_critic=100, nb_steps_warmup_actor=100,
                  random_process=random_process, gamma=.99, target_model_update=1e-3)
agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])


try:
    agent.load_weights('ddpg_{}_weights.h5f'.format(ENV_NAME))
except (OSError):
    logger.warning("File not found")


n = 0
while True:
    n += 1
    logger.info ('Iteration #{}'.format(n))

    train_history = agent.fit(env, nb_steps=1000, visualize=False, verbose=1, nb_max_episode_steps=200)

    # After training is done, we save the final weights.
    agent.save_weights('ddpg_{}_weights.h5f'.format(ENV_NAME), overwrite=True)

    # Save memory
    pickle.dump(memory, open("memory.pkl", "wb"))

    # Finally, evaluate our algorithm for nb_episodes episodes.
    test_history = agent.test(env, nb_episodes=nb_episodes, visualize=False, nb_max_episode_steps=200)


    #loading weights and model and logging taken from:
    #https://github.com/olavt/gym_co2_ventilation/blob/master/examples/test_keras_rl_continious.py
    train_rewards = train_history.history['episode_reward']
    test_rewards = test_history.history['episode_reward']
    for i in range(0, nb_episodes):
        episode_logger.info('{},{},{}'.format(((n - 1)*nb_episodes + i + 1),train_rewards[i],test_rewards[i]))

