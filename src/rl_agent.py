import numpy as np
import tensorflow as tf
import tensorflow_probability as tfp
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense, Lambda, concatenate, Conv2D, MaxPooling2D, Flatten, \
    BatchNormalization, GlobalAveragePooling2D
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.initializers import RandomUniform
from keras.applications.efficientnet import EfficientNetB0
from matplotlib import pyplot as plt


class Actor(Model):
    def __init__(self, action_dim, action_bound):
        super(Actor, self).__init__()

        self.action_dim = action_dim
        self.action_bound = action_bound
        self.std_bound = [1e-2, 1]

        self.e_net = EfficientNetB0(include_top=False, weights=None)
        self.flat = Flatten()

        self.x1 = Dense(400, activation='relu')
        self.x2 = Dense(20, activation='relu')

        self.h1 = Dense(80, activation='relu')
        self.h2 = Dense(20, activation='relu')
        self.mu = Dense(action_dim, activation='tanh', kernel_initializer=RandomUniform(-1e-3, 1e-3))
        self.std = Dense(action_dim, activation='softplus', kernel_initializer=RandomUniform(-1e-3, 1e-3))
        self.ac_d = Dense(action_dim, activation='softmax', kernel_initializer=RandomUniform(-1e-3, 1e-3))

    def call(self, state_img, state_dist):
        x1 = self.e_net(state_img)
        x1 = self.flat(x1)
        x1 = self.x1(x1)
        x2 = self.x2(state_dist)
        h = concatenate([x1, x2], axis=-1)

        x = self.h1(h)
        x = self.h2(x)
        mu = self.mu(x)
        std = self.std(x)
        ac_d = self.ac_d(x)

        mu = Lambda(lambda x: x * self.action_bound)(mu)

        std = tf.clip_by_value(std, self.std_bound[0], self.std_bound[1])

        return mu, std, ac_d

    def sample_normal(self, mu, std, ac_d):
        normal_prob = tfp.distributions.Normal(mu, std)
        action_c = normal_prob.sample()
        action_c = tf.clip_by_value(action_c, -self.action_bound, self.action_bound)
        log_pdf_c = normal_prob.log_prob(action_c)

        dist = tfp.distributions.Categorical(probs=ac_d)
        action_d = dist.sample()
        prob_d = ac_d
        log_pdf_d = tf.math.log(prob_d + (1e-8))

        return action_c, action_d, log_pdf_c, log_pdf_d, prob_d


class SACagent(object):

    def __init__(self):
        self.GAMMA = 0.95
        self.BATCH_SIZE = 64
        self.BUFFER_SIZE = 30000
        self.ACTOR_LEARNING_RATE = 0.0001
        self.CRITIC_LEARNING_RATE = 0.001
        self.TAU = 0.0001
        self.ALPHA = 0.5
        self.ALPHA_D = 0.5

        self.state_dim_img = (84, 84, 3)
        self.state_dim_dist = (1,)
        self.action_dim = 20
        self.action_bound = 35

        self.episodes = []
        self.actor_loss = []
        self.critic1_loss = []
        self.critic2_loss = []
        self.actor_loss_avg = []
        self.critic1_loss_avg = []
        self.critic2_loss_avg = []
        self.ep_temp = 0

        self.actor = Actor(self.action_dim, self.action_bound)

        state_in_img = Input(self.state_dim_img)
        state_in_dist = Input(self.state_dim_dist)

        self.actor(state_in_img, state_in_dist)

        self.actor.summary()

    def get_action(self, state_img, state_dist):
        mu, std, ac_d = self.actor(state_img, state_dist)
        action_c, action_d, _, _, _ = self.actor.sample_normal(mu, std, ac_d)
        return action_c.numpy()[0], action_d.numpy()[0]

    def load_weights(self, path):
        self.actor.load_weights(path + 'actor_e_net_a.h5')
