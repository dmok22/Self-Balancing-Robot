import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class AnimationPlot:

    def animate(self, i, dataLists, ser):
        ser.write(b'g')  # Transmit the char 'g' to receive the Arduino data point
        arduinoData_string = ser.readline().decode('ascii').strip()  # Read and decode data, remove newline/extra spaces

        try:
            x, y, z = map(float, arduinoData_string.split(','))  # Split and convert to float
            dataLists[0].append(x)  # Add x value
            dataLists[1].append(y)  # Add y value
            dataLists[2].append(z)  # Add z value

        except ValueError:  # Handle errors if data format is incorrect
            pass

        # Fix the list size to keep only the last 50 points
        dataLists[0] = dataLists[0][-50:]
        dataLists[1] = dataLists[1][-50:]
        dataLists[2] = dataLists[2][-50:]

        ax.clear()  # Clear last data frame
        
        self.getPlotFormat()
        ax.plot(dataLists[0], label="filted_angle")  # Plot X values
        ax.plot(dataLists[1], label="acc_angle")  # Plot Y values
        ax.plot(dataLists[2], label="gry_angle")  # Plot Z values
        ax.legend(loc="upper right")  # Show legend for different axis

    def getPlotFormat(self):
        ax.set_ylim([-100, 100])  # Set Y axis limit of plot
        ax.set_title("Arduino 3 angle Data")  # Set title of figure
        ax.set_ylabel("Value") 
        ax.set_xlabel("Last 50 labels")  # Set title of x axis

# Create empty lists for storing X, Y, and Z data points
dataLists = [[], [], []]  

fig = plt.figure()  # Create Matplotlib figure
ax = fig.add_subplot(111)  # Add subplot to main figure

realTimePlot = AnimationPlot()

ser = serial.Serial("COM5", 9600)  # Establish Serial object with COM port and BAUD rate
time.sleep(2)  # Time delay for Arduino Serial initialization 

# Matplotlib Animation Function that updates the plot with real-time data
ani = animation.FuncAnimation(fig, realTimePlot.animate, frames=100, fargs=(dataLists, ser), interval=25) 

plt.show()  # Keep the Matplotlib plot persistent until closed
ser.close()  # Close Serial connection when plot is closed
