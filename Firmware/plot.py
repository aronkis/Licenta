import serial
from time import sleep
from time import perf_counter
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import warnings

warnings.filterwarnings("ignore")

ser = serial.Serial(
        port='COM23',\
        baudrate=19200,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=0
    )

print("connected to: " + ser.portstr)

# Create figure for plotting
fig = plt.figure(figsize=(8, 8))
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)
xs = []
xs1 = []
ys = []
ys1 = []
startTime = -1
count1, count2 = 0, 0
pwmAverage, speedAverage = 0, 0

# This function is called periodically from FuncAnimation
# TODO: ADD PWM DRAWING
# TODO: ADD VOLTAGE DRAWING
def animate(i, xs, xs1, ys, ys1):
    global startTime, count1, count2, speedAverage, pwmAverage
    while ser.inWaiting():
        try:
            line = ser.readline().decode("utf-8")
            line = line[line.find("=") + 1:]
        except:
            pass
        try:
            if (int(line) > 1000):
                if (startTime == -1):
                    startTime = perf_counter()
                # Add x and y to lists
                speedAverage += int(line)
                count1 += 1
                if (count1 == 10):
                    xs.append(round((perf_counter() - startTime), 2))
                    ys.append(speedAverage // count1)
                    count1 = 0
                    speedAverage = 0
                
            if (int(line) < 300 and int(line) > 25):
                # Add x and y to lists
                pwmAverage += (float(line) * 0.005)
                count2 += 1
                if (count2 == 10):
                    xs1.append(round((perf_counter() - startTime), 2))
                    ys1.append(pwmAverage / count2)
                    count2 = 0
                    pwmAverage = 0
        except:
            pass

    # Limit x and y lists to 20 items
    minLen = min(len(xs), len(ys), len(ys1), 50)
    xs = xs[-60:]
    ys = ys[-60:]
    xs1 = xs1[-60:]
    ys1 = ys1[-60:]
    
    # Draw x and y lists
    ax1.clear()
    ax2.clear()
    ax1.plot(xs, ys)
    ax2.plot(xs1, ys1)
    # Adjust the axes' limits: [xmin, xmax, ymin, ymax]
    ax1.set_ylim([0, 12000])
    ax1.set_ylabel('Rotational Speed (RPM)')
    ax1.set_xticklabels(xs, rotation=45, ha='right')
    ax2.set_ylim([0, 1.25])
    ax2.set_ylabel('PWM')
    ax2.set_xticklabels(xs, rotation=45, ha='right')
    # Format plot
    plt.suptitle('Motor Stats')
    
    # Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, xs1, ys, ys1), interval=1/1000)
plt.show()
        
ser.close()
