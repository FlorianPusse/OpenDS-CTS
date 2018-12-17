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
import struct
import threading

import scipy
import scipy.ndimage
import skimage  # install scikit-image
import skimage.draw
import matplotlib.pyplot as plt
import skimage.transform

# Setting the training parameters
y = .990  # Discount factor on the target Q-values
path = "HybridCheckpoints"  # The path to save our model to.
h_size = 256  # The size of the final convolutional layer before splitting it into Advantage and Value streams.
var_end_size = 256
time_per_step = 1  # Length of each step used in gif creation
num_pedestrians = 4
num_angles = 5
num_actions = 3  # acceleration_type
image_input_size = 100 * 100 * 3
history_size = 2 * 128
observation_size = 12
load_model = True
enableTraining = False
learning_rate = 0.00015
decay = 0.99
momentum = 0.0
epsilon = 0.1

factor = 0.20
snippedSize = 100
multiplyer = factor * 10
halfSize = snippedSize // 2
car_length = 4.25
car_width = 1.7
adapted_length = car_length * multiplyer
adapted_width = car_width * multiplyer
costMapOriginal = plt.imread('../LearningAssets/combinedmapSimple.png')
costMapRescaled = skimage.transform.rescale(costMapOriginal, factor, multichannel=True, anti_aliasing=True,
                                            mode='reflect')
costMap = costMapRescaled

class SinCosLookupTable:

    def __init__(self):
        self.discretization = 0.005
        self.cosT = [None] * int(np.ceil((2 * np.pi) / self.discretization))
        self.sinT = [None] * int(np.ceil((2 * np.pi) / self.discretization))

        i = 0
        while i * self.discretization < 2 * np.pi:
            self.cosT[i] = np.cos(i * self.discretization)
            self.sinT[i] = np.sin(i * self.discretization)
            i += 1

    def sin(self, radAngle):
        return self.sinT[int(radAngle / self.discretization)]

    def cos(self, radAngle):
        return self.cosT[int(radAngle / self.discretization)]


lookupTable = SinCosLookupTable()

"""
float tempX = x - centerX;
float tempZ = z - centerZ;

float rotatedX = tempX * lookupTable.cos(theta) - tempZ * lookupTable.sin(theta);
float rotatedY = tempX * lookupTable.sin(theta) + tempZ * lookupTable.cos(theta);

return new float[]{rotatedX + centerX,rotatedY + centerZ};
"""

points = np.empty([4, 2])
points[0, :] = (- adapted_length / 2.0, - adapted_width / 2.0)
points[1, :] = (+ adapted_length / 2.0, - adapted_width / 2.0)
points[2, :] = (+ adapted_length / 2.0, + adapted_width / 2.0)
points[3, :] = (- adapted_length / 2.0, + adapted_width / 2.0)


def getCornerPositions(centerX, centerZ, theta):
    tmp = points - (centerX, centerZ)
    cos = lookupTable.cos(theta)
    sin = lookupTable.sin(theta)

    tmp1 = np.empty(points.shape)
    tmp1[:, 0] = tmp[:, 0] * cos - tmp[:, 1] * sin
    tmp1[:, 1] = tmp[:, 0] * sin + tmp[:, 1] * cos

    tmp1 += (centerX, centerZ)

    return tmp1

def getCornerPositionsSimple(theta):
    # shape: num_samples x 4 x 2
    repeatedPoints = np.repeat(points[np.newaxis, ...], theta.shape[0], axis=0)

    cos = np.cos(theta)
    sin = np.sin(theta)

    cos = np.tile(cos,(4,1)).transpose()
    sin = np.tile(sin,(4,1)).transpose()

    tmp1 = np.empty(repeatedPoints.shape)

    tmp1[:, :, 0] = repeatedPoints[:,:, 0] * cos - repeatedPoints[:,:, 1] * sin + halfSize
    tmp1[:, :, 1] = repeatedPoints[:,:, 0] * sin + repeatedPoints[:,:, 1] * cos + halfSize

    return tmp1


"""
cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - ((car_width + sideMargin)/mapResolution) / 2.0f});
cornerPositions.add(new float[]{x + ((car_length + frontMargin)/mapResolution) / 2.0f, z - ((car_width + sideMargin)/mapResolution) / 2.0f});
cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + ((car_width + sideMargin)/mapResolution) / 2.0f});
cornerPositions.add(new float[]{x + ((car_length + frontMargin)/mapResolution) / 2.0f, z + ((car_width + sideMargin)/mapResolution) / 2.0f});
"""

def getObsParallel(image, allObsData):
    samples = allObsData.shape[0]
    allObsData *= multiplyer
    allObsData[:, 2:4] /= multiplyer

    coordinate = (allObsData[:,0:2] - halfSize).round().astype(int)
    coordinate[coordinate < 0] = 0

    coordinateMax = coordinate + snippedSize

    result = np.empty([samples, 100, 100, 3])

    for iterator in range(samples):
        result[iterator, :, :] = image[coordinate[iterator, 1] : coordinateMax[iterator, 1], coordinate[iterator, 0]: coordinateMax[iterator, 0]]

    cornerPositions = getCornerPositionsSimple(allObsData[:, 2])

    for iterator in range(samples):
        rr, cc = skimage.draw.polygon(cornerPositions[iterator, :, 1], cornerPositions[iterator, :, 0], shape=[100, 100, 3])
        result[iterator, rr, cc, :] = (1.0, 0, 0)

        for i in range(num_pedestrians):
            if allObsData[iterator, 4 + (2 * i)] != 0 or allObsData[iterator, 4 + (2 * i) + 1] != 0:
                rr, cc = skimage.draw.circle(allObsData[iterator, 4 + (2 * i) + 1] - allObsData[iterator, 1] + halfSize,
                                             allObsData[iterator, 4 + (2 * i)] - allObsData[iterator, 0] + halfSize, 2, shape=[100, 100, 3])
                result[iterator, rr, cc, :] = (0, 0, 1.0)

    """
    plt.imshow(result[0,:,:])
    plt.show()
    """

    result.shape = (samples, 100 * 100 * 3)

    return result

def getObs(image, obsData):
    resizedX = obsData[0] * multiplyer
    resizedZ = obsData[1] * multiplyer

    x = int(round(max(resizedX - halfSize, 0)))
    z = int(round(max(resizedZ - halfSize, 0)))

    result = np.array(image[z:z + snippedSize, x:x + snippedSize])
    cornerPositions = getCornerPositions(0, 0, obsData[2]) + halfSize

    rr, cc = skimage.draw.polygon(cornerPositions[:, 1], cornerPositions[:, 0], shape=result.shape)
    result[rr, cc, :] = (1.0, 0, 0)

    """
    plt.imshow(result)
    plt.show()

    g2.draw(new Line2D.Float(f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor, f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor));
    g2.draw(new Line2D.Float(f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor, f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor));
    g2.draw(new Line2D.Float(f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor, f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor));
    g2.draw(new Line2D.Float(f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor, f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor));

    """

    for i in range(num_pedestrians):
        if obsData[4 + (2 * i)] != 0 or obsData[4 + (2 * i) + 1] != 0:
            rr, cc = skimage.draw.circle(obsData[4 + (2 * i) + 1] * multiplyer - resizedZ + halfSize,
                                         obsData[4 + (2 * i)] * multiplyer - resizedX + halfSize, 2, shape=result.shape)
            result[rr, cc, :] = (0, 0, 1.0)

    """
    global new_im
    if new_im:
        plt.imshow(result)
        plt.show()
        new_im = False
    """

    result.shape = (100 * 100 * 3,)

    return result

class InitNetwork:
    def __init__(self, rnn_cell, myScope):
        # The network recieves a frame from the game, flattened into an array.
        # It then resizes it and processes it through four convolutional layers.

        self.trainLength = tf.placeholder(dtype=tf.int32)
        # We take the output from the final convolutional layer and send it to a recurrent layer.
        # The input must be reshaped into [batch x trace x units] for rnn processing,
        # and then returned to [batch x units] when sent through the upper levles.
        self.batch_size = tf.placeholder(dtype=tf.int32, shape=[])

        self.scalarInput = tf.placeholder(shape=[None, image_input_size], dtype=tf.float32)

        self.imageIn = tf.reshape(self.scalarInput, shape=[-1, 100, 100, 3])
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
            inputs=self.conv3, num_outputs=h_size,
            kernel_size=[9, 9], stride=[1, 1], padding='VALID',
            biases_initializer=None, scope=myScope + '_conv4')

        self.flattened = slim.flatten(self.conv4)

        self.weights = tf.get_variable(shape=[h_size, history_size // 2], initializer=tf.glorot_normal_initializer(),
                                       name=myScope + '_w')
        self.biases = tf.get_variable(shape=[history_size // 2], initializer=tf.zeros_initializer, name=myScope + '_b')
        self.updatedInput = tf.nn.relu(tf.matmul(self.flattened, self.weights) + self.biases)

        self.updatedReshaped = tf.reshape(self.updatedInput, [self.batch_size, self.trainLength, history_size // 2])

        self.state_in = cell.zero_state(self.batch_size, tf.float32)
        self.rnn, self.rnn_state = tf.nn.dynamic_rnn(
            inputs=self.updatedReshaped, cell=rnn_cell, dtype=tf.float32, initial_state=self.state_in,
            scope=myScope + '_rnn')
        self.rnn1 = tf.reshape(self.rnn, shape=[-1, history_size // 2])

        self.weightso1 = tf.get_variable(shape=[history_size // 2, 64], initializer=tf.glorot_normal_initializer(),
                                         name=myScope + '_wo1')
        self.biaseso1 = tf.get_variable(shape=[64], initializer=tf.zeros_initializer, name=myScope + '_bo1')
        self.updatedInputOut = tf.nn.relu(tf.matmul(self.rnn1, self.weightso1) + self.biaseso1)

        self.weightsActionOut = tf.get_variable(shape=[64, num_actions], initializer=tf.glorot_normal_initializer(),
                                                name=myScope + '_wActionOut')
        self.biasesActionOut = tf.get_variable(shape=[num_actions], initializer=tf.zeros_initializer,
                                               name=myScope + '_bActionOut')
        self.actionValues = tf.matmul(self.updatedInputOut, self.weightsActionOut) + self.biasesActionOut
        self.actionProbabilities = tf.nn.softmax(self.actionValues)
        self.predictedAction = tf.argmax(self.actionProbabilities, 1)

        self.weightsValueOut = tf.get_variable(shape=[64, 1], initializer=tf.glorot_normal_initializer(),
                                               name=myScope + '_wValueOut')
        self.biasesValueOut = tf.get_variable(shape=[1], initializer=tf.zeros_initializer, name=myScope + '_bValueOut')
        self.predictedValue = tf.squeeze(tf.matmul(self.updatedInputOut, self.weightsValueOut) + self.biasesValueOut,
                                         axis=[1])

        self.despot_policy = tf.placeholder(shape=[None, num_actions], dtype=tf.float32)
        self.crossEntropy = tf.nn.softmax_cross_entropy_with_logits_v2(labels=self.despot_policy,
                                                                       logits=self.actionProbabilities)

        self.real_val = tf.placeholder(tf.float32, [None], name='real_val')
        self.td_error = tf.square(self.predictedValue - self.real_val)

        self.vars = tf.trainable_variables()
        self.lossL2 = tf.add_n([tf.nn.l2_loss(v) for v in self.vars
                                if 'b' not in v.name]) * 0.0005

        self.loss = self.td_error + self.crossEntropy + self.lossL2
        self.global_step = tf.Variable(0, trainable=False, name='step')

        self.opt = tf.train.RMSPropOptimizer(
            learning_rate=learning_rate,
            decay=decay,
            momentum=momentum,
            epsilon=epsilon)

        self.train_op = self.opt.minimize(self.loss, global_step=self.global_step)


class experience_buffer:
    def __init__(self, buffer_size=50):
        self.buffer = []
        self.buffer_size = buffer_size
        self.num_entries = 0

    def num_entries(self):
        return len(self.buffer)

    def add(self, experience):
        if len(self.buffer) + 1 >= self.buffer_size:
            self.buffer[0:(1 + len(self.buffer)) - self.buffer_size] = []
        self.buffer.append(experience)

    def sample(self):
        return self.buffer[np.random.randint(0, len(self.buffer))]


total = ""

HOST = ''
PORT = 1246


class train_connector(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.total = ""
        self.state = None
        self.lastAction = -1
        self.conn = None
        self.addr = None

        self.initialized = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.sock.bind((HOST, PORT))
        except socket.error as msg:
            print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
            sys.exit()

        print('Training connector: Bound to port ' + str(PORT) + '...')

        self.sock.listen()
        print('Training connector: Socket now listening...')

        self.conn, self.addr = self.sock.accept()
        print('Training connector: Connection Accepted...')

        self.initialized = True

    def parse(self, tmp):
        if not tmp:
            return None

        tmp = tmp.split(";")

        assert len(tmp) == 4

        arr = None
        try:
            # vector<float> real_values;
            real_values = np.array([float(x) for x in tmp[0].split(",")])

            N = len(real_values) - 1
            for i in range(1, N):
                real_values[N - i] += y * real_values[N - i + 1]

            real_values[0] += y * real_values[1]

            # vector<float*> despot_policy;
            despot_policy = np.array([float(x) for x in tmp[1].split(",")])
            despot_policy.shape = (-1, num_actions)
            # vector<float*> observations;
            observations = np.array([float(x) for x in tmp[2].split(",")])
            observations.shape = (-1, observation_size)
            # vector<float*> histories;
            histories = np.array([float(x) for x in tmp[3].split(",")])
            histories.shape = (-1, history_size)
            histories = np.split(histories, 2, 1)

            arr = {'despot_policy': despot_policy, 'real_values': real_values,
                   'observations': observations, 'histories': histories}
        except (IndexError, ValueError) as e:
            return None

        return arr

    def receiveMessage(self):
        convertedBytes = ""
        while not (convertedBytes and convertedBytes[-1] == '\n'):
            try:
                receivedBytes = self.conn.recv(1024)

                if receivedBytes == 0:
                    time.sleep(0.005)
                    continue

                convertedBytes = receivedBytes.decode('utf-8')
                self.total += convertedBytes
            except OSError as e:
                print(e)
                time.sleep(5)

        tmp_split = self.total.split("\n")

        parsed = self.parse(tmp_split[-1])
        if parsed is None:
            self.total = tmp_split[-1]
            self.state = self.parse(tmp_split[-2])
            assert (self.state is not None)
        else:
            self.total = ""
            self.state = parsed

        return self.state

    def run(self):
        total_episodes = 0

        while True:
            if not self.initialized:
                time.sleep(5)
                continue

            self.receiveMessage()
            print('Finished episode ' + str(total_episodes) + ' after '
                  + str(self.state['observations'].shape[0]) + ' steps, reward ' + str(self.state['real_values'][0]))

            message_tmp = getObsParallel(costMap, self.state['observations'])

            self.state['observations'] = message_tmp

            buf.add(self.state)

            if enableTraining:
                train_op, predicted_values, td_error = sess.run(
                    [network.train_op, network.predictedValue, network.td_error],
                    feed_dict={network.scalarInput: self.state['observations'], network.trainLength: 1,
                               network.real_val: self.state['real_values'],
                               network.despot_policy: self.state['despot_policy'],
                               network.state_in: self.state['histories'],
                               network.batch_size: self.state['observations'].shape[0]})

                """
                if total_episodes > 100:
                    for _ in range(5):
                        sample = buf.sample()

                        sess.run(network.train_op,
                                 feed_dict={network.scalarInput: sample['observations'], network.trainLength: 1,
                                            network.real_val: sample['real_values'],
                                            network.despot_policy: sample['despot_policy'],
                                            network.state_in: sample['histories'],
                                            network.batch_size: sample['observations'].shape[0]})
                """

            total_episodes += 1

            if enableTraining and total_episodes % 20 == 0 and total_episodes != 0:
                saver.save(sess, path + '/model-' + str(total_episodes) + '.cptk')
                print("Saved Model " + str(total_episodes))


class connectorServer(threading.Thread):
    def __init__(self, number):
        threading.Thread.__init__(self)
        self.path = "/tmp/python_unix_sockets_example" + str(number)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

        if os.path.exists(self.path):
            os.remove(self.path)

        try:
            self.sock.bind(self.path)
        except socket.error as msg:
            print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
            sys.exit()

        print('Connector: Bound to port...')

        self.sock.listen()

        print('Connector: Socket now listening...')

    def run(self):
        while True:
            conn, addr = self.sock.accept()
            print('Connector: Connection Accepted...')

            connectorFull(conn).start()


class connectorFull(threading.Thread):

    def __init__(self, conn):
        threading.Thread.__init__(self)

        self.total = b""
        self.state = None

        self.conn = conn

    def simpleParse(self, data, num_states):
        try:
            lstm_state1 = np.array(data[0:history_size // 2])
            lstm_state1 = np.tile(lstm_state1, num_states)
            lstm_state1.shape = (num_states, history_size // 2)

            lstm_state2 = np.array(data[history_size // 2: history_size])
            lstm_state2 = np.tile(lstm_state2, num_states)
            lstm_state2.shape = (num_states, history_size // 2)

            # (np.zeros([1, h_size + var_end_size]), np.zeros([1, h_size + var_end_size]))

            obs = np.array(data[history_size:])
            obs.shape = (-1, 12)

            arr = {'terminal': True, 'lstm_state': (lstm_state1, lstm_state2), 'obs': obs}

            return arr
        except (IndexError, ValueError) as e:
            return None

    def receiveMessage(self):
        while True:
            receivedBytes = self.conn.recv(4)
            if len(receivedBytes) == 4:
                break

        assert len(receivedBytes) == 4

        num_states = int(round(struct.unpack('f', receivedBytes)[0]))

        totalLen = (history_size + observation_size * num_states) * 4

        while len(self.total) < totalLen:
            try:
                receivedBytes = self.conn.recv(totalLen - len(self.total))

                if receivedBytes == 0:
                    continue

                self.total += receivedBytes
            except OSError as e:
                print(e)
                time.sleep(5)

        assert len(self.total) == totalLen
        num_elements = totalLen // 4
        format_string = str(num_elements) + 'f'
        data = struct.unpack(format_string, self.total)

        self.state = self.simpleParse(data, num_states)
        self.total = b""

        return self.state

    def buildBinaryMessage(self, state, action, value):
        res = b""

        for i in range(len(action)):
            res += state[0][i, :].astype('f').tostring()
            res += state[1][i, :].astype('f').tostring()
            res += struct.pack('2f', float(action[i]), value[i])

        return res

    def sendBinaryMessage(self, m):
        while True:
            try:
                self.conn.sendall(m)
                break
            except socket.error as e:
                print(e)
                time.sleep(5)

    def one_hot(self, i):
        if i == -1:
            i = 1

        return [1 if i == index else 0 for index in range(num_actions)]

    def run(self):
        while True:
            message = self.receiveMessage()
            processed_input = getObsParallel(costMap, message['obs'])

            # print(message)
            #  arr = { 'terminal': True, 'lstm_state': (lstm_state1, lstm_state2), 'obs': data[history_size:]}

            action, value, updated_state = sess.run(
                [network.predictedAction, network.predictedValue, network.rnn_state],
                feed_dict={network.scalarInput: processed_input, network.trainLength: 1,
                           network.state_in: message['lstm_state'], network.batch_size: message['obs'].shape[0]})

            self.sendBinaryMessage(self.buildBinaryMessage(updated_state, action, value))


new_im = True


class image_connector(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.total = ""
        self.path = "/tmp/python_unix_sockets_image"
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

        if os.path.exists(self.path):
            os.remove(self.path)

        try:
            self.sock.bind(self.path)
        except socket.error as msg:
            print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
            sys.exit()

        print('ImageSocket: Bound to port...')

        self.sock.listen()

        print('ImageSocket: Socket now listening...')

        self.conn, self.addr = self.sock.accept()
        print('Training connector: Connection Accepted...')

        self.initialized = True

    def parse(self, tmp):
        if not tmp:
            return None

        tmp = tmp.split(";")

        assert len(tmp) == 3

        try:
            goal_position = [float(x) for x in tmp[0].split(",")]
            obstacle = [float(x) for x in tmp[1].split(",")]
            waypoints = np.array([float(x) for x in tmp[2].split(",")])
            waypoints.shape = (-1, 2)

            arr = {'goal_position': goal_position, 'waypoints': waypoints, 'obstacle': obstacle}
        except (IndexError, ValueError) as e:
            return None

        return arr

    def receiveMessage(self):
        convertedBytes = ""
        while not (convertedBytes and convertedBytes[-1] == '\n'):
            try:
                receivedBytes = self.conn.recv(1024)

                if receivedBytes == 0:
                    time.sleep(0.005)
                    continue

                convertedBytes = receivedBytes.decode('utf-8')
                self.total += convertedBytes
            except OSError as e:
                print(e)
                time.sleep(5)

        tmp_split = self.total.split("\n")

        parsed = self.parse(tmp_split[-1])
        if parsed is None:
            self.total = tmp_split[-1]
            self.state = self.parse(tmp_split[-2])
            assert (self.state is not None)
        else:
            self.total = ""
            self.state = parsed

        return self.state

    def run(self):
        while True:
            if not self.initialized:
                time.sleep(5)
                continue

            self.receiveMessage()

            # costMapRescaled = skimage.transform.rescale(costMapOriginal, factor, multichannel=True, anti_aliasing=True, mode='reflect')
            # costMap = costMapRescaled

            # plt.imshow(costMap)
            # plt.show()

            costMapTmp = np.copy(costMapRescaled)

            self.state['waypoints'] *= multiplyer
            self.state['waypoints'] = self.state['waypoints'].round().astype(int)

            for i in range(1, self.state['waypoints'].shape[0]):
                rowOld = self.state['waypoints'][i - 1, :]
                row = self.state['waypoints'][i, :]
                rr, cc = skimage.draw.line(rowOld[1], rowOld[0], row[1], row[0])
                costMapTmp[rr, cc] = (0, 1.0, 0)

            rr, cc = skimage.draw.circle(self.state['goal_position'][1] * multiplyer,
                                         self.state['goal_position'][0] * multiplyer, 5, shape=costMapTmp.shape)
            costMapTmp[rr, cc, :] = (0, 1.0, 0)

            rr, cc = skimage.draw.circle(self.state['obstacle'][1] * multiplyer,
                                         self.state['obstacle'][0] * multiplyer, 5, shape=costMapTmp.shape)
            costMapTmp[rr, cc, :] = (0, 0, 0)

            global costMap
            costMap = costMapTmp

            global new_im
            new_im = True


buf = experience_buffer()

server = connectorServer(0)
train_connection = train_connector()
image_connection = image_connector()

tf.reset_default_graph()
# cell = tf.contrib.rnn.LSTMCell(num_units=history_size // 2, state_is_tuple=True)
cell = tf.contrib.rnn.LSTMBlockCell(num_units=history_size // 2)
network = InitNetwork(cell, 'main')

saver = tf.train.Saver(max_to_keep=30)
init = tf.global_variables_initializer()

if not os.path.exists(path):
    os.makedirs(path)

gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)

with tf.Session(config=tf.ConfigProto(gpu_options=gpu_options)) as sess:
    if load_model:
        print ('Loading Model...')
        ckpt = tf.train.get_checkpoint_state(path)
        saver.restore(sess, ckpt.model_checkpoint_path)

    else:
        sess.run(init)

    sess.graph.finalize()

    merged = tf.summary.merge_all()
    train_writer = tf.summary.FileWriter('./train', sess.graph)

    server.start()
    train_connection.start()
    image_connection.start()

    server.join()
