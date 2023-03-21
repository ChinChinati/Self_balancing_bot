# Self-balancing Two-wheeled Robot using PID Controller and MPU6050 IMU
This repository contains the code and documentation for a self-balancing two-wheeled robot built using a PID controller and an MPU6050 Inertial Measurement Unit (IMU).

### Project Overview
The goal of this project is to build a robot that can balance on two wheels without falling over. To achieve this, we are using a PID controller to control the motors and an MPU6050 IMU to measure the angle of the robot.

The robot is built using the following components:

+ Arduino Uno microcontroller board
+ MPU6050 IMU sensor
+ L298N motor driver module
+ Two DC motors
+ Two wheels
+ Chassis
+ Battery pack

The PID controller algorithm is implemented in the Arduino code, which reads the angle data from the MPU6050 sensor and calculates the motor control signals to keep the robot balanced.

### How it Works
The MPU6050 IMU sensor measures the angle of the robot using its gyroscope and accelerometer. The Arduino reads this data and calculates the error between the desired angle (0 degrees) and the actual angle of the robot. The error signal is then used to adjust the motor control signals to keep the robot balanced.

The PID controller uses three terms to adjust the motor control signals: proportional (P), integral (I), and derivative (D). The proportional term adjusts the control signal in proportion to the error signal. The integral term takes into account the accumulated error over time, and the derivative term measures how quickly the error is changing.

By tuning the PID parameters, we can adjust the behavior of the robot and make it more or less sensitive to changes in the angle.

### Project Development

+ **Simulink Simulation**: The first step was to test the PID controller algorithm in a simulation environment to see how it would perform in controlling the balance of a two-wheeled robot. For this, a Simulink model was created, which included the PID controller block and a model of the two-wheeled robot. The Simulink model was used to test different PID gain values and to observe the behavior of the robot.

+ **Tuning the PID Gains**: The next step was to tune the PID gains to achieve stable and responsive control of the robot. This involved adjusting the P, I, and D gain values in the PID controller algorithm to find the optimal values that would keep the robot balanced and prevent it from tipping over. This process involved a lot of trial and error, as different gain values had different effects on the robot's behavior.

+ **Building the Hardware Robot**: Once the PID controller algorithm was working well in simulation, the next step was to build the hardware robot. This involved assembling the hardware components, which included the Arduino Uno microcontroller board, MPU6050 IMU sensor, L298N motor driver module, two DC motors, two wheels, chassis, and battery pack. The components were connected according to the wiring diagram provided in the project documentation.

+ **Programming the Arduino**: The next step was to program the Arduino Uno microcontroller board to implement the PID controller algorithm and control the motors based on the angle data from the MPU6050 sensor. The Arduino code was written in the Arduino IDE and uploaded to the microcontroller board using a USB cable.

+ **Testing and Calibration**: The final step was to test the robot and calibrate it to achieve optimal performance. This involved observing the behavior of the robot and adjusting the PID gain values as necessary to achieve stable and responsive control. It also involved making adjustments to the physical components of the +robot, such as the weight distribution and the wheel size, to improve its stability and maneuverability.

Overall, the project involved a combination of simulation, programming, and hardware assembly, as well as a lot of trial and error to find the optimal PID gain values and physical adjustments to achieve a self-balancing two-wheeled robot.
