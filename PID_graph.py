import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class AnimationPlot:

    def animate(self, i, dataLists, ser):
        try:
            arduinoData_string = ser.readline().decode('ascii').strip()
            print(f"Raw data: '{arduinoData_string}'")  # Debug print

            # Expecting two comma-separated float values
            x, y = map(float, arduinoData_string.split(','))
            dataLists[0].append(x)  # filted_angle
            dataLists[1].append(y)  # acc_angle

        except ValueError:
            print(f"Failed to parse: '{arduinoData_string}'")  # Debug print for errors
            return

        # Keep only last 50 points for display
        dataLists[0] = dataLists[0][-50:]
        dataLists[1] = dataLists[1][-50:]

        ax.clear()
        self.getPlotFormat()
        ax.plot(dataLists[0], label="filt_angle")
        ax.plot(dataLists[1], label="acc_angle")
        ax.legend(loc="upper right")

    def getPlotFormat(self):
        ax.set_ylim([-100, 100])  # Adjust this based on your actual range
        ax.set_title("Real-Time Angle Data from Arduino")
        ax.set_ylabel("Angle (deg)")
        ax.set_xlabel("Sample (last 50)")

# --- Setup Section ---
dataLists = [[], []]  # [filt_angle[], acc_angle[]]
fig = plt.figure()
ax = fig.add_subplot(111)
realTimePlot = AnimationPlot()

# Change COM port and baud rate to match your Arduino setup
ser = serial.Serial("COM5", 9600)
time.sleep(2)  # Wait for Arduino to initialize

# Start animation
ani = animation.FuncAnimation(fig, realTimePlot.animate, fargs=(dataLists, ser), interval=25)
plt.tight_layout()
plt.show()

# Cleanup on close
ser.close()
