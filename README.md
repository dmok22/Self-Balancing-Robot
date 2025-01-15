# Angle Sensing with Arduino Nano BLE Sense IMU

## ELEC 391 Electrical Engineering Design Studio II

### Introduction

Let us consider an inverted pendulum (which will become your self-balanced robot later) that we want to balance using a certain control method (e.g., PID). This requires real-time measurements of the deviation angle **θ** (theta) that the pendulum exhibits in a non-equilibrium state. This assignment involves using both the accelerometer and the gyroscope sensors to measure this angle.

---

### 1. Background

The accelerometer and gyroscope sensors on the Arduino Nano BLE Sense IMU (BMI270_BMM150) provide two independent methods to compute the angle **θ**. From a control theory perspective, the angle **θ** is the error **e(t)** and is zero at equilibrium (vertical position).

#### Computing Angles with Accelerometer Readings

An accelerometer measures gravitational acceleration on its vertical axis. If the accelerometer's vertical axis is not aligned with gravity (**θ ≠ 0**), the angle can be computed as:

\[
θ = \arctan\left(\frac{a_x}{a_y}\right)
\]

For small tilt angles:

\[
θ \approx \frac{a_x}{a_y}
\]

where \(a_x, a_y\) are accelerometer readings on the horizontal and vertical axes, respectively.

#### Advantages:
- Zero-mean error.
- No measurement bias accumulation.

#### Disadvantages:
- High variance in Gaussian noise.
- Noise from mechanical components (e.g., motors).

#### Computing Angles with Gyroscope Readings

The gyroscope measures angular speed (\(\dot{θ}\)) in degrees/second. To compute the angle of rotation:

\[
θ = θ_0 + g_z \cdot \Delta t
\]

where \(g_z\) is the gyroscope reading for the z-axis and \(Δt\) is the sampling time.

#### Advantages:
- Less noise over short time periods.

#### Disadvantages:
- Accumulates bias over time due to continuous integration.

---

### 2. Complementary Filter

To combine the benefits of both sensors, a complementary filter is used. It provides accurate short-term gyroscope readings and long-term accelerometer stability. The filter output is calculated as:

\[
θ_n = k \cdot θ_{n-1} + θ_{\text{gyro},n} + (1-k) \cdot θ_{\text{acc},n}
\]

where:
- \(k\) is a weight (0 to 1).
- \(θ_n\) is the filter output at time step \(n\).
- \(θ_{\text{gyro},n}\) is the angle from gyroscope readings.
- \(θ_{\text{acc},n}\) is the angle from accelerometer readings.

Start with \(k = 0.01\) for accelerometer and \(k = 0.99\) for gyroscope.

---

### Tasks

#### Task 1 – Data Plotting [1 Mark]
Plot real-time sensor data from Arduino Nano 33 BLE Sense using Python (or similar). The Arduino IDE Serial Plotter is not allowed. Deliverables:
- `task1.ino` (Arduino code)
- `task1.py` (Python code)

#### Task 2 – Computing Angles with Accelerometer Readings [1 Mark]
Write Arduino code to collect accelerometer data and compute the tilt angle. Display this angle using Task 1's plotting method. Deliverables:
- `task2.ino` (Arduino code)
- `task2.py` (Python code)

#### Task 3 – Computing Angles with Gyroscope Readings [1 Mark]
Repeat Task 2 using gyroscope data. Deliverables:
- `task3.ino` (Arduino code)
- `task3.py` (Python code)

#### Task 4 – Calculate Angles using a Complementary Filter [2 Marks]
Use the complementary filter to compute the tilt angle. Tune the \(k\) coefficient for accuracy. Deliverables:
- `task4.ino` (Arduino code)
- `task4.py` (Python code)

---

### Requirements
1. Demonstrate angle readings at **0°**, **±15°**, and **±30°**.
2. Submit all files as a single `.zip` archive named `group_yy.zip`, containing eight files:
   - `task1.ino` and `task1.py`
   - `task2.ino` and `task2.py`
   - `task3.ino` and `task3.py`
   - `task4.ino` and `task4.py`
