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


def readPort(queue,radar):
    # regex expression to read 3 or  digit numbers till \n
    regex_pattern = r"[0-9][0-9]{0,3}"
    with serial.Serial(radar.port, radar.baudrate, timeout=1) as ser:
        while True:
            byte_array = ser.readline()
            #byte_val = byte_array.decode('ascii')
            z = re.findall(regex_pattern, str(byte_array))
            #convert from string to integer
            if len(z)>0:
                val = int(z[0])
                # add to queue
                queue.put(val)


def Plot(queue, radar):
    plt.figure()

    ctr = 0
    temp = 0
    start_time = time()
    while True:
        if ctr == temp:
            start_time = time()
            ctr = ctr + 1
        if not(queue.empty()):
            if len(radar.buffer) < radar.N:
                radar.buffer.append(queue.get())
            else:
                yf1 = np.multiply(radar.window, fftshift(fft(radar.buffer)) / radar.N)
                #adding 1e-9 to avoid log10(zero) error
                yf1 = yf1 + np.array([1e-9 * radar.N])
                yf = 10 * np.log10(abs(yf1))
                # ignoring frequency bin at frequency 0
                yf[int(radar.N / 2)] = 0
                plt.clf()
                #print(queue.qsize())
                plt.subplot(2, 1, 1)
                plt.plot(radar.time_axis, np.array(radar.buffer))
                plt.title("DOPPLER RADAR time domain output "+str(ctr))
                plt.ylabel("Amplitude")
                plt.xlabel("time (s)")
                plt.ion()
                plt.show()
                radar.clear_buf()
                plt.subplot(2, 1, 2)
                plt.plot(radar.vel_axis, yf)
                plt.title("DOPPLER RADAR frequency domain output "+str(ctr))
                plt.ylabel("PSD")
                plt.xlabel("velocity (m/s)")
                # plt.draw()
                #plt.cla()
                plt.ion()
                plt.xlim(radar.xlim)
                plt.ylim(radar.ylim)
                plt.show()
                plt.pause(0.5)
                print("Time for loop: " + str(time() - start_time) + " s")
                temp = ctr
                start_time = time()

class radar_params():
    def __init__(self):
        #controlling the duration to sample for 1 DFT block
        self.N = 4096
        ##controlling sampling frequency of ADC
        self.clk = 16e6
        self.prescaler = 256
        self.trip = 40
        self.port = "COM4"
        self.baudrate = 115200
        # ADC buffer - holds N time samples sampled every  ts seconds corresponding to T duration
        self.buffer = []
        #read settings from file
        self.read_settings()
        #calculate radar dependent parameters
        self.calculate_radar_params()
        #calculate axes
        self.get_axes()

    def clear_buf(self):
        self.buffer = []

    def read_settings(self):
        with open('settings.txt') as f:
            settings_dict = json.load(f)
        assert settings_dict["N"], "Number of time sample (N) not found in settings.txt"
        assert settings_dict["clk"], "Arduino clock frequency (clk) not found in settings.txt"
        assert settings_dict["prescaler"], "Arduino prescaler value not found in settings.txt"
        assert settings_dict["trip"], "Arduino trip value (trip) not found in settings.txt"
        assert settings_dict["f_c"], "Radar carrier frequency (f_c) not found in settings.txt"
        assert settings_dict["port"], "Arduino Serial COM port not (port) found in settings.txt"
        assert settings_dict["baudrate"], "Arduino Serial COM port baudrate (baudrate) not found in settings.txt"
        self.N = int(settings_dict["N"])
        self.clk = float(settings_dict["clk"])
        self.prescaler = float(settings_dict["prescaler"])
        self.trip = float(settings_dict["trip"])
        self.f_c = float(settings_dict["f_c"])
        self.port = settings_dict["port"]
        self.baudrate = int(settings_dict["baudrate"])
        self.xlim = [float(f) for f in settings_dict["xlim"].split(',')]
        self.ylim = [float(f) for f in settings_dict["ylim"].split(',')]
        print("*********FILE SETTINGS********************************")
        print("N: ", self.N)
        print("clk ", self.clk)
        print("prescaler ", self.prescaler)
        print("baudrate: ", self.baudrate)
        print("port: ", self.port)
        print("xlim: ", self.xlim)
        print("ylim: ", self.ylim)
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
    q = Queue()
    # p1 is the process that reads data from Serial port and adds it to queue
    p1 = Process(name='p1', target=readPort, args=(q, radar))
    # p2 is the process that gets data fom p1 and does signal processing and plotting
    p2 = Process(name='p2', target=Plot, args=(q, radar))
    p1.start()
    p2.start()
