import sys
import datetime
import csv

# Serial imports
import serial
from enum import Enum

# Analysis imports
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Analysis parameters
wavelength = 632.8e-9                           # 632.8nm
temp = 293.15                                   # 20C
kb = 1.38e-23                                   # Boltzmann constant J/K
theta = np.pi / 2                               # Measurement angle
n_solv = int(sys.argv[1])
eta_solv = int(sys.argv[2])                     # Pa * s
q = 4*np.pi/wavelength*n_solv*np.sin(theta/2)   # scattering vector
time_interval = 0.0000156                       # Microseconds
measure_period = float(sys.argv[3])             # Milliseconds
periods_averaged = int(sys.argv[4])             # Number of cycles to average ACF
num_samp_per_period = int(measure_period / time_interval)

def processData(data):
    """This method is an adaptation of https://github.com/etiennerognin/OpenDLS"""
    num_samp = len(data)
    num_division = int(num_samp / num_samp_per_period)
    if periods_averaged < num_division:
        num_div = periods_averaged
    else:
        num_div = num_division
    
    # Autocorrelation from FFT
    for i in num_div:
        split = data[(num_samples_per_ms * i) : (num_samples_per_ms - 1) * (i + 1)]
        s_hat = np.ftt.rfft(split)
        cumulative_autocorr += np.fft.irfft(s_hat*np.conj(s_hat), num_samples_per_ms)

    # Ignore noisy data below 10% of autocorrelation range
    time = np.linspace(time_interval, 1, num=num_samp_per_ms, endpoint=True)
    cutoff_val = (max(cumulative_autocorr)-min(cumulative_autocorr)) * 0.1 + min(cumulative_autocorr)
    cutoff_idx = np.where((cumulative_autocorr > cutoff_val) == False)[0][0]
    tau = 1e-3*time[:cutoff]
    ydata = cumulative_autocorr[:cutoff] / num_div
    
    # Fitting function
    def g(tau, a, b, c): 
        return a*np.ones_like(tau) + b*np.exp(-c*tau)

    a0 = 
    b0 = 1
    c0 = 2*q**2*kb*T/(18*1e-7*eta_solv)
    popt, pcov = curve_fit(g, tau, ydata, p0=[a0, b0, c0])
    (a, b, c) = popt

    # Sample parameters
    D = c/(2*q**2)                      # Diffusion coefficient
    Rh = kb*temp/(6*np.pi*D*eta_solv)   # Radius
    psize = 2*Rh*1e9                    # Particle size in nanometers
    
    # Extract error
    perr = np.sqrt(np.diag(pcov))
    dc_over_c = perr[2]/c
    psizeerr = dc_over_c*psize

    printout = """
    Diffusion coeff : %s
    Particle size   : %s
    Size error      : %s
    Cycle length    : %s
    Cycles averaged : %s
    """

    print(printout, D, psize, psizeerr, measure_period, periods_averaged)
    print("Saving data and figures...")

    timestamp = now.strftime("%Y%M%D_%H%M%S")
    numpy.savetxt("DLS_" + timestamp + "ACF.csv", numpy.dstack(((tau, ydata))[0][:], fmt="%f", delimiter=",")

    plt.semilogx(tau, ydata, 'k.', label='ACF Data')
    plt.semilogx(tau, g(tau, a, b, c), 'r-', label='fit: Diffusion = %.2E m^2/s, Particle size=%d nm, ' % (D,psize))
    plt.semilogx(tau, g(tau, a, b, 0.83*c), 'r--', label='+/- 20% in size')
    plt.semilogx(tau, g(tau, a, b, 1.25*c), 'r--')
    plt.xlabel('Tau (s)')
    plt.ylabel('Auto-correlation')
    plt.legend()
    plt.savefig("DLS_" + timestamp + "_Figure.png")

    print("Save complete!")
    

# Serial parameters
class Status(Enum):
    ACQUIRE = "A"
    BUSY    = "B"
    READY   = "R"
    SEND    = "S

ser = serial.Serial(sys.argv[5], 115200)
ser.open()

# Main Program
response = ser.read()
sleep(.1)
if(~ser.in_waiting):
    print("Port activity not detected.")
    exit()
    
while(True):
    command = input("(A)cquire/(S)end Data: ")
    if(command ~= Status.ACQUIRE or Status.SEND):
        print("Command not understood.")
        exit()
    ser.flush()
    ser.write(command.encode('utf-8'))
    response = Status.READY
    while(response == Status.READY):
        response = ser.read().decode('utf-8')

    if(command == Status.ACQUIRE):
        while(~ser.in_waiting):
            pass
        print("Data acquired on sensor board.")

    if(command == Status.SEND):
        data = []
        response[0] = ser.read().decode('utf-8')
        while(response[0].decode('utf-8') ~= Status.READY):
            data.append(response)
            response = ser.read(2)
        print("Data received.")
        processData(data)
        print("Data processed.")
