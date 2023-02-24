# STM32H7 Flight Controller
## NUCLEO-H723ZG based UAVs

This flight controller sets up the motor control pins as outputs and initialize the MPU6050 gyroscope and accelerometer sensor and the Adafruit BMP280 barometric pressure sensor. The MPU6050 is calibrated and its offsets are stored, and the BMP280 is started.

Sensor values from the MPU6050 and BMP280 sensors are read to calculate PID control signals for pitch, roll, yaw, and altitude. The PID control signals are then used to set the motor speeds, and the previous error values are stored for use in the next iteration. The motor speeds are constrained to the range of 0-255.

For RC control, PWM signals are mapped to control signals for the drone flight controller, which are then converted to setpoints for the PID controller. 

The code includes an emergencyShutdown() function with the following features:
- A check for low battery voltage is included to prevent the drone from running on insufficient power.
- A check for excessive pitch or roll angles is included to prevent the drone from entering unstable flight conditions.
- A check for excessive PID control signals is included to prevent the drone from exceeding safe motor speeds.

# Features to implement

~~• Add remote control functionality to use with RC transmitters and 2.4 GHz receivers.~~

- Add an accelerometer-based altitude hold feature: The current code uses a barometer to measure altitude, which can be affected by changes in air pressure. Using an accelerometer to measure altitude instead can provide more accurate and responsive altitude control.

- Implement a failsafe mode: In case of communication loss with the RC controller or other malfunctions, it's important to have a failsafe mode that will safely bring the drone back to the ground. This could involve automatically descending the drone or returning it to a home location.

- Add a gyro-based yaw control feature: The current code uses a simple PID controller to control yaw, but this can be improved by adding a complementary filter that combines both gyro and accelerometer measurements to achieve more stable yaw control.

- Implement a log file for flight data: Recording flight data can be useful for troubleshooting issues and improving performance over time. The log file could include sensor readings, control signals, and other relevant flight data.

- Add a remote kill switch: In case of emergencies, it can be useful to have a way to immediately shut off the motors from the RC controller. This could be implemented using a simple switch that sends a signal to the microcontroller to shut off the motors.

- Use an external interrupt for RC control: Currently, the RC control signals are read continuously in the main loop of the code, which can introduce latency and slow down other parts of the code. Using an external interrupt to read the RC control signals can improve response time and overall performance.

# License
If you have any questions or suggestions, please feel free to contact me:
awf.wis@gmail.com
