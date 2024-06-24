import sys
import serial
import warnings
from time import perf_counter
import matplotlib.pyplot as plt
import matplotlib.animation as animation

warnings.filterwarnings("ignore")

SpeedXAxis = []
SpeedYAxis = []
SpeedYAxisLimits = [i for i in range(0, 13000, 1000)]

PWMXAxis = []
PWMYAxis = []
PWMYAxisLimits = [(i/10) for i in range(0, 13)]

startTime = -1
count1, count2 = 0, 0 # used for oversampling 
pwmAverage, speedAverage = 0, 0

serialPort = 'COM23'
baudRate = 19200
plotLimit = 50

def getArgumentValue(argument):
    return argument[argument.find("=") + 1:]

# This function is called periodically from FuncAnimation
def animate(i, ser, ax1, ax2, SpeedXAxis, PWMXAxis, SpeedYAxis, PWMYAxis, plotLimit):

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
                # Add x and y to the rotation speed list
                speedAverage += int(line)
                count1 += 1
                if (count1 == 10):
                    SpeedXAxis.append(round((perf_counter() - startTime), 2))
                    SpeedYAxis.append(speedAverage // count1)
                    count1 = 0
                    speedAverage = 0
                
            if (int(line) < 300 and int(line) > 25):
                # Add x and y to the PWM list
                pwmAverage += (float(line) * 0.005)
                count2 += 1
                if (count2 == 10):
                    PWMXAxis.append(round((perf_counter() - startTime), 2))
                    PWMYAxis.append(pwmAverage / count2)
                    count2 = 0
                    pwmAverage = 0
        except:
            pass

    # Limit x and y lists to 20 items
    SpeedXAxis = SpeedXAxis[-plotLimit:]
    SpeedYAxis = SpeedYAxis[-plotLimit:]
    PWMXAxis = PWMXAxis[-plotLimit:]
    PWMYAxis = PWMYAxis[-plotLimit:]
    
    # Draw x and y lists
    ax1.clear()
    ax2.clear()
    ax1.plot(SpeedXAxis, SpeedYAxis)
    ax2.plot(PWMXAxis, PWMYAxis)
    
    # Formatting the plots
    ax1.set_ylim([0, 12000])
    ax1.set_yticks(SpeedYAxisLimits)
    ax1.set_ylabel('Rotational Speed [RPM]')
    ax1.set_xlabel('Time [s]')
    ax1.set_xticklabels(SpeedXAxis, rotation=45, ha='right')
    
    ax2.set_ylim([0, 1.25])
    ax2.set_yticks(PWMYAxisLimits)
    ax2.set_ylabel('Duty Cycle [%]')
    ax2.set_xlabel('Time [s]')
    ax2.set_xticklabels(SpeedXAxis, rotation=45, ha='right')

    plt.suptitle('Motor Stats', size='xx-large')

def main():
    if (len(sys.argv) > 1):
        for arg in sys.argv:
            if ("plotLimit" in arg):
                plotLimit = int(getArgumentValue(arg))
            if ("serialPort" in arg):
                serialPort = getArgumentValue(arg)
            if ("baudRate" in arg):
                baudRate = int(getArgumentValue(arg))

    # Connecting to the given Serial Port
    ser = serial.Serial(
            port = serialPort,\
            baudrate = baudRate,\
            parity = serial.PARITY_NONE,\
            stopbits = serial.STOPBITS_ONE,\
            bytesize = serial.EIGHTBITS,\
            timeout = 0
        )
    print("connected to: " + ser.portstr)
    print("plotLimit = ", plotLimit)

    # Create figure for plotting
    fig = plt.figure(figsize = (8, 8))
    ax1 = fig.add_subplot(2, 1, 1)
    ax2 = fig.add_subplot(2, 1, 2)
    plt.subplots_adjust(top = 0.945, bottom = 0.08, hspace = 0.3)
        
    # Set up plot to call animate() function periodically
    ani = animation.FuncAnimation(fig, animate, fargs = (ser, ax1, ax2, SpeedXAxis, PWMXAxis, SpeedYAxis, PWMYAxis, plotLimit), interval = 1/1000)
    plt.show()
            
    ser.close()
    plt.close()

if __name__ == "__main__":
    main()