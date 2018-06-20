#!/usr/bin/env python

import os
import time
import struct
import math
import numpy as np
import scipy as sp
import pymoos
import matplotlib.pyplot as plt
from matplotlib.mlab import window_hanning,specgram
from matplotlib.colors import LogNorm

class pPySpectrogram(object):
    def __init__(self):
        self.moos_app_name = 'PySpectrogram'
        self.time_warp = 1
        self.server_host = 'localhost'
        self.server_port = 9000
        self.freq_sampling = 37500.0
        self.num_samples = 16000
        self.num_hydrophones = 5
        self.aco_data = np.zeros((self.num_samples, self.num_hydrophones))  #array that holds the acoustic data currently being processed
        self.aco_data_latest = None                                         #array that holds new acoustic data while it waits to be processed
        self.aco_new = False                                                #flag to indicate that new acoustic data has been read
        self.extents = None

        ''' Initialize Python-MOOS Communications '''
        self.mooscomms = pymoos.comms()
        self.mooscomms.set_on_connect_callback(self.moos_on_connect)
        
        self.mooscomms.add_active_queue('cbf_queue', self.moos_on_new_acoustic_binary_data) #let's use a queue callback instead to handle distinct messages
        self.mooscomms.add_message_route_to_active_queue('cbf_queue', 'DAQ_BINARY_DATA')    #route 'DAQ_BINARY_DATA' messages to moos_on_new_acoustic_binary_data function

        self.mooscomms.run(self.server_host, self.server_port, self.moos_app_name)
        pymoos.set_moos_timewarp(self.time_warp)

    def moos_on_connect(self):
        ''' On connection to MOOSDB, register for desired MOOS variables (allows for * regex) e.g. register('variable', 'community', 'interval')
        self.mooscomms.register('NODE_*_PING','NODE_*',0) '''
        self.mooscomms.register('DAQ_BINARY_DATA', 0)       #register for acoustic binary data
        return True

    def moos_on_new_acoustic_binary_data(self, msg):
        ''' Queue callback function to handle 'DAQ_BINARY_DATA' messages specifically; it has the same accessors a regular mail callback:
        msg.trace(), msg.time(), msg.name(), msg.key(), msg.is_name(), msg.source(), msg.is_double(), msg.double(), msg.double_aux(),
        msg.is_string(), msg.string(), msg.is_binary(), msg.binary_data(), msg.binary_data_size(), msg.mark_as_binary() '''
        msg_binary = msg.binary_data()  # remember - iMCC1608FS DAQ data is sent as binary float (32-bit)
        try:
            self.aco_data_latest = np.frombuffer(msg_binary, dtype=np.float32).reshape((self.num_samples, self.num_hydrophones))   # much faster than struct.unpack
            self.aco_new = True
        except Exception, e:
            print 'Error occurred attempting to unpack acoustic binary data:'
            print str(e)
        return True

    def get_specgram(self,signal):
        arr2D,freqs,bins = specgram(signal,window=window_hanning,Fs=self.freq_sampling,NFFT=256,noverlap=128)
        return arr2D,freqs,bins

    def run(self):
        while True:
            if self.aco_new:
                self.aco_data[:] = self.aco_data_latest

                arr2D,freqs,bins = self.get_specgram(self.aco_data[:,0])
                if self.extents is None:
                    self.extents = (freqs[0],freqs[-1],bins[0],bins[-1])
                    img.set_extent(self.extents)
                    ax.invert_yaxis()
                img.set_array(arr2D.T)
                img.set_norm(LogNorm(vmin=np.min(arr2D),vmax=np.max(arr2D)))
                img.axes.figure.canvas.draw()
                plt.show()

                self.aco_new = False
                
            time.sleep(0.01)

plt.ion()
plt.figure(figsize=(8, 6))
ax = plt.gca()
fig = plt.gcf()
img = ax.imshow(np.zeros((513,257)), aspect='auto', cmap='jet', interpolation='nearest', norm=LogNorm(vmin=1e-16,vmax=1e-5))
plt.xlabel('Frequency (Hz)')
plt.ylabel('Time (s)')
if __name__ == '__main__':
    pyspec = pPySpectrogram()
    pyspec.run()
