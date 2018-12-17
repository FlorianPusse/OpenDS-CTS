# Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import os
import socket
import time

if sys.version_info >= (3,0):
    from queue import Queue
else:
    from Queue import Queue

import numpy as np
#import scipy.misc as misc

from Config import Config

class connector:

    def __init__(self,id):
        self.total = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(("localhost", 4000 + id + Config.AGENTS))
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

        if (len(arr['obs']) == 4 + Config.NUM_PEDESTRIANS*2 and len(arr['map']) == Config.IMAGE_HEIGHT*Config.IMAGE_WIDTH*3):
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
                        self.sock.connect(("localhost", 1245))
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
                        self.sock.connect(("localhost", 1234))
                        break
                    except OSError:
                        print("Timeout during connect ")
                        time.sleep(5)

    def getState(self):
        if self.state is None:
            self.receiveMessage()

        tmp = np.concatenate((self.state['map'],[self.state['obs'][2]]), axis=0)
        tmp = np.concatenate((tmp,[self.lastAction]), axis=0)
        tmp = np.concatenate((tmp,[self.state['reward']]),axis=0)

        #image, vel, action, reward
        return tmp

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

        return [1 if i == index else 0 for index in range(Config.NUM_ACTIONS)]

class Environment:

    def __init__(self,id):
        self.id = id
        self.nb_frames = Config.STACKED_FRAMES
        self.frame_q = Queue(maxsize=self.nb_frames)
        self.previous_state = None
        self.current_state = None
        self.total_reward = 0
        self.connection = connector(id)
        self.receivedTerminal = False

    def _action(*entries):
        return np.array(entries, dtype=np.intc)


    def is_running(self):
        return not self.receivedTerminal
    
    @staticmethod
    def _rgb2gray(rgb):
        return np.dot(rgb[..., :3], [0.299, 0.587, 0.114])

    @staticmethod
    def _preprocess(image):
        #image = Environment._rgb2gray(image)
        #image = misc.imresize(image, [Config.IMAGE_HEIGHT, Config.IMAGE_WIDTH], 'bilinear')
        image = image.astype(np.float32) / 1.
        return image

    def _get_current_state_no_stacking(self):
        if not self.frame_q.full():
            return None  # frame queue is not full yet.
        return np.array(list(self.frame_q.queue)[0])

    def _get_current_state(self):
        if not self.frame_q.full():
            return None  # frame queue is not full yet.
        x_ = [np.array(i) for i in list(self.frame_q.queue)]
        x_ = np.concatenate(x_, axis=2)
        #x_ = np.array(self.frame_q.queue)
        #x_ = np.transpose(x_, [1, 2, 3, 0])  # move channels
        return x_

    def _update_frame_q(self, frame):
        if self.frame_q.full():
            self.frame_q.get()
        self.frame_q.put(frame)

        # the state is no longer just the image, but a concatenation of
        # image and auxiliary inputs. We can't use the same _preprocess()
        #image = Environment._preprocess(frame)
        #self.frame_q.put(image)

    def get_num_actions(self):
        return Config.NUM_ACTIONS

    def reset(self):
        self.total_reward = 0
        self.frame_q.queue.clear()
        self.connection.reset()
        self.receivedTerminal = False
        self._update_frame_q(self.connection.getState())
        self.previous_state = self.current_state = None

    def step(self, action):
        reward, is_running = self.connection.step(action)
        self.receivedTerminal = not is_running
        self.total_reward += reward
        self.previous_state = self.current_state
        
        if is_running:
          observation = self.connection.getState()
          self._update_frame_q(observation)
          self.current_state = self._get_current_state_no_stacking() 
          
        return reward, is_running
