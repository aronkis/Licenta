import serial
from time import sleep
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# This function is called periodically from FuncAnimation
# TODO: ADD PWM DRAWING
# TODO: ADD VOLTAGE DRAWING
def animate(i, xs, ys):

    while ser.inWaiting(): 
        line = ser.readline().decode("utf-8")
        line = line[line.find("=") + 1:]
        try:
            if (int(line) > 1000):
                print(int(line))
                print(type(int(line)))
                # Add x and y to lists
                xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
                ys.append(int(line))
                i += 1
        except:
            pass

    # Limit x and y lists to 20 items
    xs = xs[-25:]
    ys = ys[-25:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)
    # Adjust the axes' limits: [xmin, xmax, ymin, ymax]
    ax.set_ylim([0, 10000])
    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Motor Speed')
    plt.ylabel('Rotational Speed (RPM)')
    
    # Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=50/1000)
plt.show()
        
ser.close()
