# Doppler Radar Code
# Gaurav Duggal Oct 17th 2020
import serial
import matplotlib.pyplot as plt
from numpy.fft import fft, fftshift
import re
from multiprocessing import Process, Queue
import numpy as np
from time import time
import json
import PyQt5

def readPort(q1,radar):
    # regex expression to read 3 or  digit numbers till \n
    regex_pattern = r"[0-9][0-9]{0,3}"
    with serial.Serial(radar.port, radar.baudrate, timeout=1) as ser:
        while True:
            byte_array = ser.readline()
            #byte_val = byte_array.decode('ascii')
            z = re.findall(regex_pattern, str(byte_array))
            #convert from string to integer
            if len(z)>0 :
              #print(z)
              val = (5/1023)*int(z[0])
              # add to queue
              q1.put(val)

def Plot(q1, radar):
    fig = plt.figure()
    ctr = 0
    temp = 0
    start_time = time()
    buf = []
    while True:
        if ctr == temp:
            start_time = time()
            ctr = ctr + 1

        if not(q1.empty()):
            if q1.qsize() > (radar.STFT_nfft + 10) :
                for i in range(radar.STFT_nfft):
                    while len(buf) > radar.N:
                        buf.pop(0)
                    buf.append(q1.get())
                if (len(buf) > radar.N):
                    # STFT algorithm
                    plt.subplot(3,1,1)
                    plt.plot(radar.time_axis, np.array(buf[0:radar.N]))
                    plt.grid()
                    #0 to 5 V
                    plt.ylim([0, 5])
                    # o to buffer size in seconds
                    plt.xlim([0, radar.ts*radar.N])
                    plt.title("DOPPLER RADAR time domain output "+str(ctr))
                    plt.ylabel("Amplitude (V)")
                    plt.xlabel("time (s)")
                    plt.ion()
                    plt.show()
                    plt.subplot(3, 1, 2)
                    Pxx, freq= plt.psd(buf[0:radar.N], radar.N, radar.fs*radar.lamb/2,window=np.hamming(radar.N),
                                       pad_to=radar.N*2, detrend='mean')
                    plt.subplot(3, 1, 3)
                    maxp = 10*np.log10(np.max(abs(Pxx)))
                    print(maxp)

                    plt.specgram(buf[0:radar.N],mode="psd", NFFT=radar.STFT_nfft, scale='dB', window=np.hamming(radar.STFT_nfft),
                                 pad_to=radar.STFT_zero_padding, detrend='mean', noverlap=radar.STFT_no_overlap,
                                 Fs=radar.fs*radar.lamb/2,vmin=maxp-23, vmax=maxp)
                    plt.ylim([0, 5])

                    plt.show()
                    plt.pause(0.001)
                    if q1.qsize()>int(radar.N/radar.STFT_nfft)*radar.N :
                        buf=[]
                    plt.clf()
                    print("Time for loop: " + str(time() - start_time) + " s")
                    temp = ctr
                    start_time = time()
                    print(q1.qsize())

class radar_params():
    def __init__(self):
        #read settings from file
        self.read_settings()
        #calculate radar dependent parameters
        self.calculate_radar_params()
        #calculate axes
        self.get_axes()


    def checkKey(self, dict, key):
        if key in dict.keys():
            return True
        else:
            return False


    def read_settings(self):
        with open('settings.txt') as f:
            settings_dict = json.load(f)
        assert self.checkKey(settings_dict, "N"), "Number of time sample (N) not found in settings.txt"
        assert self.checkKey(settings_dict, "clk"), "Arduino clock frequency (clk) not found in settings.txt"
        assert self.checkKey(settings_dict, "prescaler"), "Arduino prescaler value not found in settings.txt"
        assert self.checkKey(settings_dict, "trip"), "Arduino trip value (trip) not found in settings.txt"
        assert self.checkKey(settings_dict, "f_c"), "Radar carrier frequency (f_c) not found in settings.txt"
        assert self.checkKey(settings_dict, "port"), "Arduino Serial COM port not (port) found in settings.txt"
        assert self.checkKey(settings_dict, "baudrate"), "Arduino Serial COM port baudrate (baudrate) not found in settings.txt"
        assert self.checkKey(settings_dict, "STFT_nfft"), "STFT Algorithm time window not set properly"
        assert self.checkKey(settings_dict, "STFT_zero_padding"), "STFT Algorithm zero padding not set properly"
        assert self.checkKey(settings_dict, "STFT_no_overlap"), "STFT Algorithm number of overlap time samples not set properly"
        self.N = int(settings_dict["N"])
        self.clk = float(settings_dict["clk"])
        self.prescaler = float(settings_dict["prescaler"])
        self.trip = float(settings_dict["trip"])
        self.f_c = float(settings_dict["f_c"])
        self.port = settings_dict["port"]
        self.baudrate = int(settings_dict["baudrate"])
        self.xlim = [float(f) for f in settings_dict["xlim"].split(',')]
        self.ylim = [float(f) for f in settings_dict["ylim"].split(',')]
        self.STFT_nfft = int(settings_dict["STFT_nfft"])
        self.STFT_zero_padding = int(settings_dict["STFT_zero_padding"])
        self.STFT_no_overlap = int(settings_dict["STFT_no_overlap"])

        print("*********FILE SETTINGS********************************")
        print("N: ", self.N)
        print("clk ", self.clk)
        print("prescaler ", self.prescaler)
        print("baudrate: ", self.baudrate)
        print("port: ", self.port)
        print("xlim: ", self.xlim)
        print("ylim: ", self.ylim)
        print("STFT_nfft: ", self.STFT_nfft)
        print("STFT_zero_padding: ", self.STFT_zero_padding)
        print("STFT_no_overlap: ", self.STFT_no_overlap)
        print("hamming winndow used in STFT")
        self.hamming = np.hamming(self.STFT_nfft)
        print("*****************************************************")

    def calculate_radar_params(self):
        self.fs = self.clk/(self.prescaler*self.trip)+1
        print("Sampling frequency of ADC: ", self.fs)
        self.ts = 1/float(self.fs)
        print("Sampleing time of ADC: ",self.ts)
        # speed of light
        self.c = 3e8
        # wavelength
        self.lamb = self.c / float(self.f_c)
        print("Wavelength: " ,self.lamb, " m")
        # N point FFT is performed i.e. we collect these samples for 1 FFT plot
        # windowing is used for suppression of sidelobes in FFT plot
        self.window = np.hamming(self.N)
        print("Hamming window used")
        # total sampling duration for 1 FFT (decides Doppler/velocity resolution)
        self.T = self.N * self.ts
        print("Sampling duration: ",self.T)
        # frequency bin width/ frequency resolution
        self.delta_f = 1 / (float(self.T))
        print("Doppler frequency resolution: ", self.delta_f, "Hz")
        # Doppler velocity resolution
        self.doppler_resolution = self.lamb * self.delta_f / 2
        print("Doppler velocity resolution: ", self.doppler_resolution, " m/s")
        #max Doppler velocity
        self.max_doppler = self.lamb*self.fs/4- self.doppler_resolution
        print("max detectable velocity: ", self.max_doppler, " m/s")


    def get_axes(self):
        self.time_axis = np.linspace(0, self.T - self.ts, self.N)
        self.freq_axis = np.linspace(-self.fs / 2, self.fs / 2 - self.delta_f, self.N)
        self.vel_axis = self.c * self.freq_axis / (2 * self.f_c)


if __name__== '__main__':
    #initialise radar parameters from settings.txt
    radar = radar_params()
    #queue used to transfer data from p1 to p2
    q1 = Queue()
    # p1 is the process that reads data from Serial port and adds it to queue
    p1 = Process(name='p1', target=readPort, args=(q1, radar))
    # p2 is the process that gets data fom p1 and does signal processing and plotting
    p2 = Process(name='p2', target=Plot, args=(q1, radar))
    p1.start()
    p2.start()
