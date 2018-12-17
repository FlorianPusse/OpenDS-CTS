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

# check python version; warn if not Python3
import os, sys
import warnings
if sys.version_info < (3,0):
    warnings.warn("Optimized for Python3. Performance may suffer under Python2.", Warning)

from Config import Config
from Server import Server

# Suppress the output from C functions
# source - http://stackoverflow.com/questions/5081657/how-do-i-prevent-a-c-shared-library-to-print-on-stdout-in-python
def redirect_stdout():
    sys.stdout.flush() # <--- important when redirecting to files
    newstdout = os.dup(1)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1)
    os.close(devnull)
    sys.stdout = os.fdopen(newstdout, 'w')

def checks():
  if Config.STACKED_FRAMES != 1:
    assert False, "Stacking of multiple frames not supported. See disentangle_obs() in NetworkVP.py"

  if Config.NUM_LSTMS != 2:
    assert False, "Architecture hard-wired for 2 stacked LSTM layers"

if __name__ == '__main__':

    # Parse arguments
    for i in range(1, len(sys.argv)):
        # Config arguments should be in format of Config=Value
        # For setting booleans to False use Config=
        x, y = sys.argv[i].split('=')
        setattr(Config, x, type(getattr(Config, x))(y))

    # Adjust configs for Play mode
    if Config.PLAY_MODE:
        print("==Play mode on==")
        #Config.AGENTS = 1
        Config.PREDICTORS = 1
        Config.TRAINERS = 1
        Config.DYNAMIC_SETTINGS = False

        Config.LOAD_CHECKPOINT = True
        Config.TRAIN_MODELS = False
        Config.SAVE_MODELS = False

    #redirect_stdout()
    #checks()
    print('===Network===')
    print('LSTM layers:', Config.NUM_LSTMS)
    print("Reward clipping %s. Clipping affects policy!"%('ENABLED' if Config.REWARD_CLIPPING else 'DISABLED'))
    print('======')
    # Start main program
    Server().main()
