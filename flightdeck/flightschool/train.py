import atexit
import logging
import os
from threading import Thread

import numpy as np
from keras import Sequential, Model
from keras.layers import Input, Flatten, Dense, Activation, Concatenate
from keras.optimizers import Adam
from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess

import cflib
from cflib.crazyflie import Crazyflie
from flightdeck.flightschool.quad_environment import CrazyflieEnvironment
from flightdeck.util import find_available_drones

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def teardown_env(env, cf):
    env.close()
    cf.close_link()


class FlightTraining:
    def __init__(self):
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

    def connect(self, link_uri):
        print('Connecting to %s.' % link_uri)
        self._cf.open_link(link_uri)

    def _train(self):
        env = CrazyflieEnvironment(self._cf)
        atexit.register(teardown_env, env, self._cf)

        np.random.seed(123)
        assert len(env.action_space.shape) == 1
        nb_actions = env.action_space.shape[0]

        # Next, we build a very simple model.
        actor = self.actor_model(env, nb_actions)
        action_input, critic = self.critic_model(env, nb_actions)

        memory = SequentialMemory(limit=1000000, window_length=1)
        random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.3)
        model_name = 'ddpg_{}_weights.h5f'.format('crazyflie')
        agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
                          memory=memory, nb_steps_warmup_critic=128, nb_steps_warmup_actor=128,
                          random_process=random_process, gamma=.99, target_model_update=1e-3)
        if os.path.exists(model_name):
            agent.load_weights(model_name)
        agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])

        try:
            agent.fit(env, nb_steps=50000, verbose=2)
            agent.test(env, nb_episodes=1)
        finally:
            agent.save_weights(model_name, overwrite=True)

    def critic_model(self, env, nb_actions):
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
        critic.summary()
        return action_input, critic

    def actor_model(self, env, nb_actions):
        actor = Sequential()
        actor.add(Flatten(input_shape=(1,) + env.observation_space.shape))
        actor.add(Dense(16))
        actor.add(Activation('relu'))
        actor.add(Dense(16))
        actor.add(Activation('relu'))
        actor.add(Dense(16))
        actor.add(Activation('relu'))
        actor.add(Dense(nb_actions, activation='sigmoid'))
        actor.summary()
        return actor

    def _connected(self, link_uri):
        print('Connected to %s. Starting training...' % link_uri)
        Thread(target=self._train).start()

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    drones = find_available_drones()
    if not drones:
        raise AssertionError('No drones found.')

    training = FlightTraining()
    training.connect(drones[0][0])
