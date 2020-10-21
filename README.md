# Doppler_Radar_RCWL-0516 Signal Processing
 
1). Connect the Arduino Pin A0 to the Doppler radar signal out
(i.e. pin 12 of RCWL-0516) refer to https://github.com/jdesbonnet/RCWL-0516/ 
2). Burn the Doppler_radar.ino to the Arduino, I used an AMTEGA 328P based Arduino UNO. 
3). Put the appropriate port settings in settings.txt in eg "port": "COM4"
4). Run main.py

The console should read:

*********FILE SETTINGS********************************
N:  4096
clk  16000000.0
prescaler  256.0
baudrate:  115200
port:  COM4
xlim:  [0.0, 5.0]
ylim:  [-0.5, 15.0]
*****************************************************
Sampling frequency of ADC:  1563.5
Sampleing time of ADC:  0.0006395906619763352
Wavelength:  0.09674298613350532  m
Hamming window used
Sampling duration:  2.619763351455069
Doppler frequency resolution:  0.3817138671875 Hz
Doppler velocity resolution:  0.018464069680143504  m/s
max detectable velocity:  37.79595063525375  m/s

The file settings contain user editable and independent parameters:

1). Arduino Parameters:
The Arduino runs timer0 at 1563 Hz and the ADC is sampled each time the
Interrupt Service Routine is called (ISR).The sampling frequency is given by:

fs = (clk)/(prescaler*trip) + 1

the xlim and ylim set the axis limits in the fourier plot generated so we can
see clearly the Doppler shifts


2). Radar Parameters:
a). The sampling frequency controls the maximum doppler frequency that we can 
measure which is given by fs/2 according to the nyquist theorem
b). The Dwell time (T) is given by N*(1/fs) and this controls the width of the
Doppler Frequency bin or Doppler frequency resolution (del_f). Doppler frequency resolution is given by the inverse of the Dwell time. 

del_f = 1/T

c). The Doppler shift velocity  resolution (del_v) given by:

del_v = del_f*lambda/2 

where lambda is the wavelength of the passband radar signal i.e. in our case 

lambda = c/f_c where f_c = 3.101 GHz



