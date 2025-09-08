import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

ser = serial.Serial('COM4', 115200)

x_co2, y_co2 = [], []
x_voc, y_voc = [], []

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

count = 0

def update(frame):
    global count
    if count < 100:
        line = ser.readline().decode().strip()
        # now = time.time()
        if "CO2" in line:
            value = float(line.split(': ')[1])
            x_co2.append(count)
            y_co2.append(value)
        elif "VOC" in line:
            value = float(line.split(': ')[1])
            x_voc.append(count)
            y_voc.append(value)
        if "CO2" not in line and "VOC" not in line:
            ax1.cla()
            ax2.cla()
            ax1.plot(x_co2, y_co2, 'g-', label='CO2 (ppm)')
            ax2.plot(x_voc, y_voc, 'b-', label='VOC (ppb)')
            ax1.set_ylim(0, 2000)  # CO2 (ppm) 
            ax2.set_ylim(0, 1000)  # VOC (ppb) 
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('CO2 (ppm)', color='g')
            ax2.set_ylabel('VOC (ppb)', color='b')
            ax2.yaxis.set_label_position("right")
            ax1.legend(loc='upper left')
            ax2.legend(loc='upper right', bbox_to_anchor=(1, 1))
            count += 1
            plt.title('Real-time CO2 and VOC Levels')
            plt.grid()
            plt.tight_layout()

ani = FuncAnimation(fig, update, interval=1)
plt.show()