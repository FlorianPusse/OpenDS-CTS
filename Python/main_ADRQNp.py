import sys
import os
import socket
import numpy as np
import random
import tensorflow as tf
import tensorflow.contrib.slim as slim
import csv
import time
from ADRQN.helper import *
import time
from datetime import datetime

# Setting the training parameters
batch_size = 32  # How many experience traces to use for each training step.
trace_length = 8  # How long each experience trace will be when training
update_freq = 1  # How often to perform a training step.
y = .995  # Discount factor on the target Q-values
startE = 1  # Starting chance of random action
endE = 0.1  # Final chance of random action
anneling_steps = 200000 #10000 # How many steps of training to reduce startE to endE.
num_episodes = 100000  # How many episodes of game environment to train network with.
pre_train_steps = 10000 # 10000  # How many steps of random actions before training begins.
load_model = True  # Whether to load a saved model.
path = "/home/anon/Documents/MasterStuff/Python/Checkpoints"  # The path to save our model to.
h_size = 512  # The size of the final convolutional layer before splitting it into Advantage and Value streams.
var_end_size = 128
max_epLength = 300  # The max allowed length of our episode.
time_per_step = 1  # Length of each step used in gif creation
summaryLength = 100  # Number of epidoes to periodically save for analysis
num_pedestrians = 4
num_angles = 3
num_actions = num_angles*3  # acceleration_type
input_size = 1 + 4 + 2 * num_pedestrians + num_actions  # angle + 4 car related statistics + 2*num_pedestrians related statistics + one-hot encoded last_action
image_input_size = 100*100*3
tau = 1
targetUpdateInterval = 10000

use_dueling = False

class Qnetwork():
    def __init__(self, h_size, rnn_cell, myScope):
        # The network recieves a frame from the game, flattened into an array.
        # It then resizes it and processes it through four convolutional layers.

        self.imageInput = tf.placeholder(shape=[None, image_input_size], dtype=tf.float32)

        self.imageIn = tf.reshape(self.imageInput, shape=[-1, 100, 100, 3])
        self.conv1 = slim.convolution2d(
            inputs=self.imageIn, num_outputs=32,
            kernel_size=[8, 8], stride=[4, 4], padding='VALID',
            biases_initializer=None, scope=myScope + '_conv1')
        self.conv2 = slim.convolution2d(
            inputs=self.conv1, num_outputs=64,
            kernel_size=[4, 4], stride=[2, 2], padding='VALID',
            biases_initializer=None, scope=myScope + '_conv2')
        self.conv3 = slim.convolution2d(
            inputs=self.conv2, num_outputs=64,
            kernel_size=[3, 3], stride=[1, 1], padding='VALID',
            biases_initializer=None, scope=myScope + '_conv3')
        self.conv4 = slim.convolution2d(
            inputs=self.conv3,num_outputs=h_size,
            kernel_size=[9,9],stride=[1,1],padding='VALID',
            biases_initializer=None,scope=myScope+'_conv4')

        self.trainLength = tf.placeholder(dtype=tf.int32)
        # We take the output from the final convolutional layer and send it to a recurrent layer.
        # The input must be reshaped into [batch x trace x units] for rnn processing,
        # and then returned to [batch x units] when sent through the upper levles.
        self.batch_size = tf.placeholder(dtype=tf.int32, shape=[])

        self.scalarInput = tf.placeholder(shape=[None, input_size], dtype=tf.float32)
        self.weights = tf.get_variable(shape=[input_size, 100], initializer=tf.glorot_normal_initializer(), name=myScope + '_w')
        self.biases = tf.get_variable(shape=[100], initializer=tf.zeros_initializer, name=myScope + '_b')
        self.updatedInput = tf.nn.relu(tf.matmul(self.scalarInput, self.weights) + self.biases)

        self.weights1 = tf.get_variable(shape=[100, var_end_size], initializer=tf.glorot_normal_initializer(), name=myScope + '_w1')
        self.biases1 = tf.get_variable(shape=[var_end_size], initializer=tf.zeros_initializer, name=myScope + '_b1')
        self.updatedInput1 = tf.nn.relu(tf.matmul(self.updatedInput, self.weights1) + self.biases1)

        self.flattened = slim.flatten(self.conv4)

        # concatenate the convolved map with the other input data
        self.concatLayers = tf.concat([self.flattened, self.updatedInput1], 1)
        self.convFlat = tf.reshape(self.concatLayers, [self.batch_size, self.trainLength, (h_size) + var_end_size])

        self.state_in = cell.zero_state(self.batch_size, tf.float32)
        self.rnn, self.rnn_state = tf.nn.dynamic_rnn(
            inputs=self.convFlat, cell=rnn_cell, dtype=tf.float32, initial_state=self.state_in, scope=myScope + '_rnn')
        self.rnn1 = tf.reshape(self.rnn, shape=[-1, h_size + var_end_size])

        if use_dueling:
            # The output from the recurrent player is then split into separate Value and Advantage streams
            self.streamA, self.streamV = tf.split(self.rnn1, 2, 1)
            self.AW = tf.Variable(tf.truncated_normal([(h_size + var_end_size) // 2, num_actions], stddev=0.1), name=myScope + "AW")
            self.VW = tf.Variable(tf.truncated_normal([(h_size + var_end_size) // 2, 1], stddev=0.1), name=myScope + "VW")
            self.Advantage = tf.matmul(self.streamA, self.AW)
            self.Value = tf.matmul(self.streamV, self.VW)

            self.Qout = self.Value + tf.subtract(self.Advantage,
                                                 tf.reduce_mean(self.Advantage, reduction_indices=1, keep_dims=True))
        else:
            self.weightso1 = tf.get_variable(initializer=tf.truncated_normal([h_size + var_end_size, num_actions], stddev=0.1), name=myScope + '_wo1')
            self.biaseso1 = tf.get_variable(shape=[num_actions], initializer=tf.zeros_initializer, name=myScope + '_bo1')

            # Then combine them together to get our final Q-values.
            self.Qout = tf.matmul(self.rnn1, self.weightso1) + self.biaseso1

        self.predict = tf.argmax(self.Qout, 1)

        # Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
        self.targetQ = tf.placeholder(shape=[None], dtype=tf.float32)
        self.actions = tf.placeholder(shape=[None], dtype=tf.int32)
        self.actions_onehot = tf.one_hot(self.actions, num_actions, dtype=tf.float32)

        self.Q = tf.reduce_sum(tf.multiply(self.Qout, self.actions_onehot), reduction_indices=1)

        self.td_error = tf.square(self.targetQ - self.Q)

        self.maskA = tf.zeros([self.batch_size, self.trainLength//2])
        self.maskB = tf.ones([self.batch_size, self.trainLength//2])
        self.mask = tf.concat([self.maskA, self.maskB], 1)
        self.mask = tf.reshape(self.mask, [-1])

        self.loss = tf.reduce_mean(self.td_error * self.mask)

        self.trainer = tf.train.RMSPropOptimizer(learning_rate=0.00020, momentum=0.95, epsilon=0.01, decay=0.95)
        self.updateModel = self.trainer.minimize(self.loss)


class experience_buffer():
    def __init__(self, buffer_size=300):
        self.buffer = []
        self.buffer_size = buffer_size

    def add(self, experience):
        if len(self.buffer) + 1 >= self.buffer_size:
            self.buffer[0:(1 + len(self.buffer)) - self.buffer_size] = []
        self.buffer.append(experience)

    def sample(self, batch_size, trace_length):
        sampled_episodes = random.sample(self.buffer, batch_size)
        sampledTraces = []
        for episode in sampled_episodes:
            point = np.random.randint(0, len(episode) + 1 - trace_length)
            sampledTraces.append(episode[point:point + trace_length])
        sampledTraces = np.array(sampledTraces)
        return np.reshape(sampledTraces, [batch_size * trace_length, 7])


total = ""


PORT = 4001

class connector:

    def __init__(self):
        self.total = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(("localhost", PORT))
        self.state = None
        self.lastAction = -1

    def reset(self):
        self.sendMessage("RESET\n")

        while self.state is None or self.state['terminal']:
            self.sendMessage("RESET\n")
            self.receiveMessage()

    def parse(self, tmp):
        if not tmp:
            return None

        tmp = tmp.split(";")

        arr = None
        try:
            arr = {'terminal': True if tmp[0] == 'true' else False, 'reward': float(tmp[1]), 'angle': float(tmp[2]),
                   'obs': [float(x) for x in tmp[3].split(",")], 'map': [float(x) for x in tmp[4].split(",")]}
        except (IndexError, ValueError) as e:
            return None

        if (len(arr['obs']) == 4 + num_pedestrians*2 and len(arr['map']) == 100*100*3):
            return arr

        return None

    def receiveMessage(self):
        convertedBytes = ""
        while not (convertedBytes and convertedBytes[-1] == '\n'):
            try:
                receivedBytes = self.sock.recv(1024)

                if receivedBytes == 0:
                    time.sleep(0.005)
                    continue

                convertedBytes = receivedBytes.decode('utf-8')
                self.total += convertedBytes
            except OSError as e:
                print(e)
                print("Try to restore connection")
                while True:
                    try:
                        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.sock.connect(("localhost", PORT))
                        break
                    except OSError as p:
                        print("Timeout during connect ")
                        time.sleep(5)

        tmp_split = self.total.split("\n")

        parsed = self.parse(tmp_split[-1])
        if parsed is None:
            self.total = tmp_split[-1]
            self.state = self.parse(tmp_split[-2])
            assert(self.state is not None)
        else:
            self.total = ""
            self.state = parsed

        return self.state

    def sendMessage(self, m):
        while True:
            try:
                self.sock.send(m.encode())
                break
            except socket.error as e:
                print(e)
                print("Try to restore connection")
                while True:
                    try:
                        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.sock.connect(("localhost", PORT))
                        break
                    except OSError:
                        print("Timeout during connect ")
                        time.sleep(5)

    # reward, is_running = self.connection.step(action)
    def step(self,action):

        # Replace noop by our noop
        if action == -1:
            action = 1

        self.sendMessage(str(action) + "\n")
        self.receiveMessage()
        self.lastAction = action

        return self.state['reward'], not self.state['terminal']

    def one_hot(self, i):
        if i == -1:
            i = 1

        return [1 if i == index else 0 for index in range(num_actions)]

tf.reset_default_graph()
# We define the cells for the primary and target q-networks
cell = tf.contrib.rnn.LSTMCell(num_units=h_size + var_end_size, state_is_tuple=True)
cellT = tf.contrib.rnn.LSTMCell(num_units=h_size + var_end_size, state_is_tuple=True)
mainQN = Qnetwork(h_size, cell, 'main')
targetQN = Qnetwork(h_size, cellT, 'target')

saver = tf.train.Saver(max_to_keep=10)
init = tf.global_variables_initializer()

trainables = tf.trainable_variables()

targetOps = updateTargetGraph(trainables, tau)

myBuffer = experience_buffer()

# Set the rate of random action decrease.
e = startE
stepDrop = (startE - endE) / anneling_steps

# create lists to contain total rewards and steps per episode
jList = []
rList = []
total_steps = 0

# Make a path for our model to be saved in.
if not os.path.exists(path):
    os.makedirs(path)

##Write the first line of the master log-file for the Control Center
with open(path + '/log.csv', 'w+') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
    wr.writerow(['Episode', 'Length', 'Reward', 'IMG', 'LOG', 'SAL'])

connection = connector()

gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)

with tf.Session(config=tf.ConfigProto(gpu_options=gpu_options)) as sess:
    if load_model == True:
        print ('Loading Model...')
        ckpt = tf.train.get_checkpoint_state(path)
        saver.restore(sess, ckpt.model_checkpoint_path)

        # TODO
        startE = 0.05
    else:
        sess.run(init)
        updateTarget(targetOps, sess)  # Set the target network to be equal to the primary network.

    sess.graph.finalize()

    merged = tf.summary.merge_all()
    train_writer = tf.summary.FileWriter('./train', sess.graph)

    for i in range(num_episodes):
        episodeBuffer = []
        # Reset environment and get first new observation

        totalEpisodeReward = 0
        connection.sendMessage("RESET\n")

        message = connection.receiveMessage()
        s = connection.one_hot(num_actions) + [message['angle']]
        s.extend(message['obs'])
        s = np.array(s)
        m = np.array(message['map'])
        connection.sendMessage("START\n")

        d = False
        rAll = 0
        j = 0
        state = (np.zeros([1, h_size + var_end_size]), np.zeros([1, h_size + var_end_size]))  # Reset the recurrent layer's hidden state
        # The Q-Network
        while j < max_epLength:
            if total_steps == pre_train_steps:
                print("Pre-train phase done.")

            j += 1
            # Choose an action by greedily (with e chance of random action) from the Q-network
            if total_steps % 10 == 0:
                if np.random.rand(1) < e or total_steps < pre_train_steps:
                    Qout, state1 = sess.run([mainQN.Qout, mainQN.rnn_state],
                                    feed_dict={mainQN.scalarInput: [s / 1.0], mainQN.imageInput: [m / 1.0], mainQN.trainLength: 1,
                                               mainQN.state_in: state, mainQN.batch_size: 1})
                    a = np.random.randint(0, num_actions)
                else:
                    Qout, a, state1 = sess.run([mainQN.Qout, mainQN.predict, mainQN.rnn_state],
                                       feed_dict={mainQN.scalarInput: [s / 1.0], mainQN.imageInput: [m / 1.0], mainQN.trainLength: 1,
                                                  mainQN.state_in: state, mainQN.batch_size: 1})
                    a = a[0]
                print(str(total_steps) + ", " + np.array_str(Qout))
            else:
                if np.random.rand(1) < e or total_steps < pre_train_steps:
                    state1 = sess.run([mainQN.rnn_state],
                                          feed_dict={mainQN.scalarInput: [s / 1.0], mainQN.imageInput: [m / 1.0], mainQN.trainLength: 1,
                                                     mainQN.state_in: state, mainQN.batch_size: 1})
                    a = np.random.randint(0, num_actions)
                else:
                    Qout, a, state1 = sess.run([mainQN.Qout, mainQN.predict, mainQN.rnn_state],
                                             feed_dict={mainQN.scalarInput: [s / 1.0], mainQN.imageInput: [m / 1.0], mainQN.trainLength: 1,
                                                        mainQN.state_in: state, mainQN.batch_size: 1})
                    a = a[0]

            connection.sendMessage(str(a) + "\n")

            message = connection.receiveMessage()
            s1 = connection.one_hot(a) + [message['angle']]
            s1.extend(message['obs'])
            s1 = np.array(s1)
            m1 = np.array(message['map'])
            r = message['reward']
            d = message['terminal']

            totalEpisodeReward += r*(y**j)
            total_steps += 1

            episodeBuffer.append(np.reshape(np.array([s, a, r, s1, d, m, m1]), [1, 7]))
            if total_steps > pre_train_steps and i > batch_size + 4:
                if e > endE:
                    e -= stepDrop

                if total_steps % targetUpdateInterval == 0:
                    print("Target network updated.")
                    updateTarget(targetOps, sess)

                if total_steps % (update_freq) == 0:
                    # Reset the recurrent layer's hidden state
                    state_train = (np.zeros([batch_size, h_size + var_end_size]), np.zeros([batch_size, h_size + var_end_size]))

                    if total_steps % (update_freq * 5) == 0:
                        start_time = datetime.now()

                    trainBatch = myBuffer.sample(batch_size, trace_length)  # Get a random batch of experiences.

                    if total_steps % (update_freq * 100) == 0:
                        time_elapsed = datetime.now() - start_time
                        print('Time elapsed for sampling from buffer (hh:mm:ss.ms) {}'.format(time_elapsed))
                        start_time = datetime.now()

                     # Below we perform the Double-DQN update to the target Q-values
                    Q1 = sess.run(mainQN.predict, feed_dict={
                        mainQN.scalarInput: np.vstack(trainBatch[:, 3] / 1.0),
                        mainQN.imageInput:  np.vstack(trainBatch[:, 6] / 1.0),
                        mainQN.trainLength: trace_length,
                        mainQN.state_in: state_train,
                        mainQN.batch_size: batch_size})
                    Q2 = sess.run(targetQN.Qout, feed_dict={
                        targetQN.scalarInput: np.vstack(trainBatch[:, 3] / 1.0),
                        targetQN.imageInput:  np.vstack(trainBatch[:, 6] / 1.0),
                        targetQN.trainLength: trace_length,
                        targetQN.state_in: state_train,
                        targetQN.batch_size: batch_size})
                    
                    end_multiplier = -(trainBatch[:, 4] - 1)
                    doubleQ = Q2[range(batch_size * trace_length), Q1]
                    targetQ = trainBatch[:, 2] + (y * doubleQ * end_multiplier)
                    # Update the network with our target values.
                    sess.run(mainQN.updateModel,
                             feed_dict={mainQN.scalarInput: np.vstack(trainBatch[:, 0] / 1.0),
                                        mainQN.imageInput: np.vstack(trainBatch[:, 5] / 1.0),
                                        mainQN.targetQ: targetQ,
                                        mainQN.actions: trainBatch[:, 1],
                                        mainQN.trainLength: trace_length,
                                        mainQN.state_in: state_train,
                                        mainQN.batch_size: batch_size})

                    if total_steps % (update_freq * 100) == 0:
                        time_elapsed = datetime.now() - start_time
                        print('Time elapsed for updating model (hh:mm:ss.ms) {}'.format(time_elapsed))

            rAll += r
            s = s1
            state = state1
            m = m1

            if d == True:
                break

        # Add the episode to the experience buffer

        if j > trace_length:
            bufferArray = np.array(episodeBuffer)
            episodeBuffer = list(zip(bufferArray))
            myBuffer.add(episodeBuffer)
            jList.append(j)
            rList.append(rAll)

        # Periodically save the model.
        if i % 150 == 0 and i != 0:
            saver.save(sess, path + '/model-' + str(i) + '.cptk')
            print ("Saved Model")
        if len(rList) % summaryLength == 0 and len(rList) != 0:
            print (total_steps, np.mean(rList[-summaryLength:]), e)
            # saveToCenter(i, rList, jList, np.reshape(np.array(episodeBuffer), [len(episodeBuffer), 5]),
            #             summaryLength, h_size, sess, mainQN, time_per_step)

        print("Finished epsiode " + str(i) + ". Total reward received after " + str(j) + " steps: " + str(totalEpisodeReward))

    saver.save(sess, path + '/model-' + str(i) + '.cptk')
