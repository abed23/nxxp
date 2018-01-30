"""
A simple example of an animated plot
"""
from scipy.interpolate import spline
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from threading import Thread
import time

fig = plt.figure()

ax = fig.add_subplot(1,1,1)


# Serial data collection
last_received = ''
ser = serial.Serial(port='\\.\COM6', baudrate=9600)

# Serial data fetching
def receiving(ser):
    global last_received

    buffer_string = ''
    while True:
        buffer_string = buffer_string + ser.read(ser.inWaiting())
        if 'L:' in buffer_string:
            lines = buffer_string.split('L:') # Guaranteed to have at least 2 entries
            last_received = lines[-2]
            #If the Arduino sends lots of empty lines, you'll lose the
            #last filled line, so you could make the above statement conditional
            #like so: if lines[-2]: last_received = lines[-2]
            buffer_string = lines[-1]

def animate(i):
    newData = last_received.split(",")
    xar = []
    yar = []
    t = 0
    if (len(newData) > 124 ):
        for eachLine in newData:
            t=t+1
            xar.append(float(t))
            yar.append(float(eachLine))
        ax.clear()

        ygradient = np.gradient(yar)
        xnew = np.linspace(min(xar), max(xar), 300)
        power_smooth = spline(xar, yar, xnew)
        power_smoothg = spline(xar, ygradient, xnew)
        ax.plot(xar, yar, 'r',  label='Gradient')
        # ax.plot(xnew, power_smoothg , 'b', label='Derivative filter')
        fig.gca().set_title("Plotting camera data")
        fig.gca().set_xlabel("Sample no.")
        fig.gca().set_ylabel("Intensity")
        fig.legend()
        ax.set_yscale('linear')
        fig.gca().set_xlim(left=0, right=127)
        fig.gca().set_ylim(bottom=-300, top=300)







Thread(target=receiving, args=(ser,)).start()
ani = animation.FuncAnimation(fig, animate, interval=20,  frames=1000)
plt.show()