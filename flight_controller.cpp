#include "mbed.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Servo.h"

// define pins for motor control
const PinName motor1Pin = PC_9;
const PinName motor2Pin = PA_8;
const PinName motor3Pin = PB_13;
const PinName motor4Pin = PA_9;

// define variables for sensor calibration
const int calibrateTime = 5000;
const float gravity = 9.81;

// define variables for PID control
const float kp = 1.0;
const float ki = 0.0;
const float kd = 0.0;

// define variables for sensor readings
MPU6050 mpu;
BMP280 bmp(I2C_SDA, I2C_SCL);
float roll, pitch, yaw, altitude;
float setpointRoll = 0.0, setpointPitch = 0.0, setpointYaw = 0.0, setpointAltitude = 0.0;
float previousErrorRoll = 0.0, previousErrorPitch = 0.0, previousErrorYaw = 0.0, previousErrorAltitude = 0.0;
float integralRoll = 0.0, integralPitch = 0.0, integralYaw = 0.0, integralAltitude = 0.0;

// define variables for RC control
Servo channel1(PB_0);
Servo channel2(PB_1);
Servo channel3(PA_7);
Servo channel4(PA_6);
int channel1Value, channel2Value, channel3Value, channel4Value;

// define variables for arming and disarming the drone
bool armed = false;
int disarmChannel = 6;  // channel for disarming the drone
int armChannel = 7;     // channel for arming the drone
int disarmValue = 1000; // PWM signal value for disarming the drone
int armValue = 2000;    // PWM signal value for arming the drone

int main() {
    // set up motor control pins as PWM outputs
    PwmOut motor1(motor1Pin);
    PwmOut motor2(motor2Pin);
    PwmOut motor3(motor3Pin);
    PwmOut motor4(motor4Pin);
    motor1.period_us(2000);
    motor2.period_us(2000);
    motor3.period_us(2000);
    motor4.period_us(2000);
    motor1.write(0.0);
    motor2.write(0.0);
    motor3.write(0.0);
    motor4.write(0.0);

    // set up I2C communication
    I2C i2c(I2C_SDA, I2C_SCL);

    // set up MPU6050 sensor
    mpu.initialize(&i2c);
    mpu.calibrateGyro();
    mpu.calibrateAccel();
    wait_ms(calibrateTime);
    mpu.getGyroOffsets();

    // set up BMP280 sensor
    bmp.init();

    // set up RC control
    channel1.calibrate(1000, 2000, 0.0, 1.0 / 1000000.0);
    channel2.calibrate(1000, 2000, 0.0, 1.0 / 1000000.0);
    channel3.calibrate(1000, 2000, 0.0, 1.0 / 1000000.0);
    channel4.calibrate(1000, 2000, 0.0, 1.0 / 1000000.0);

    // initialize the drone in a disarmed state
    armed = false;
    motor1.write(0.0);
    motor2.write(0.0);
    motor3.write(0.0);
    motor4.write(0.0);

    while(1) {
        // read sensor values
        mpu.update();
        roll = mpu.getRoll();
        pitch = mpu.getPitch();
        yaw = mpu.getYaw();
        altitude = bmp.readAltitude();

        // read RC control signals
        channel1Value = channel1.read_us();
        channel2Value = channel2.read_us();
        channel3Value = channel3.read_us();
        channel4Value = channel4.read_us();

        // check if the drone should be armed or disarmed based on RC signals
        if (channel1Value >= armValue && channel2Value >= armValue && channel3Value >= armValue && channel4Value >= armValue) {
            armed = true;
        } else if (channel1Value <= disarmValue && channel2Value <= disarmValue && channel3Value <= disarmValue && channel4Value <= disarmValue) {
            armed = false;
            motor1.write(0.0);
            motor2.write(0.0);
            motor3.write(0.0);
            motor4.write(0.0);
        }

        if (armed) {
            // map RC control signals to control signals for the drone flight controller
            float rollSignal = map(channel1Value, 1000, 2000, -255, 255);
            float pitchSignal = map(channel2Value, 1000, 2000, -255, 255);
            float yawSignal = map(channel3Value, 1000, 2000, -255, 255);
            float altitudeSignal = map(channel4Value, 1000, 2000, 0, 255);

            // convert control signals to setpoints for the PID controller
            setpointRoll = rollSignal;
            setpointPitch = pitchSignal;
            setpointYaw = yawSignal;
            setpointAltitude = altitudeSignal;

            // calculate PID control signals for pitch, roll, yaw, and altitude
            float errorRoll = setpointRoll - roll;
            float errorPitch = setpointPitch - pitch;
            float errorYaw = setpointYaw - yaw;
            float errorAltitude = setpointAltitude - altitude;
            integralRoll += errorRoll;
            integralPitch += errorPitch;
            integralYaw += errorYaw;
            integralAltitude += errorAltitude;
            float derivativeRoll = errorRoll - previousErrorRoll;
            float derivativePitch = errorPitch - previousErrorPitch;
            float derivativeYaw = errorYaw - errorYaw - previousErrorYaw;
            float derivativeAltitude = errorAltitude - previousErrorAltitude;
            pitchSignal = kp * errorPitch + ki * integralPitch + kd * derivativePitch;
            rollSignal = kp * errorRoll + ki * integralRoll + kd * derivativeRoll;
            yawSignal = kp * errorYaw + ki * integralYaw + kd * derivativeYaw;
            altitudeSignal = kp * errorAltitude + ki * integralAltitude + kd * derivativeAltitude;

            // check for excessive PID control signals
            if (abs(pitchSignal) > 255.0 || abs(rollSignal) > 255.0 || abs(yawSignal) > 255.0 || abs(altitudeSignal) > 255.0) {
                emergencyShutdown(motor1, motor2, motor3, motor4);
            }

            // set motor speeds based on PID control signals
            float motor1Speed = altitudeSignal + pitchSignal - rollSignal - yawSignal;
            float motor2Speed = altitudeSignal + pitchSignal + rollSignal + yawSignal;
            float motor3Speed = altitudeSignal - pitchSignal + rollSignal - yawSignal;
            float motor4Speed = altitudeSignal - pitchSignal - rollSignal + yawSignal;

            // constrain motor speeds to 0-1 range
            motor1Speed = constrain(motor1Speed, 0.0, 1.0);
            motor2Speed = constrain(motor2Speed, 0.0, 1.0);
            motor3Speed = constrain(motor3Speed, 0.0, 1.0);
            motor4Speed = constrain(motor4Speed, 0.0, 1.0);

            // set motor speeds
            motor1.write(motor1Speed);
            motor2.write(motor2Speed);
            motor3.write(motor3Speed);
            motor4.write(motor4Speed);

            // store previous error values
            previousErrorRoll = errorRoll;
            previousErrorPitch = errorPitch;
            previousErrorYaw = errorYaw;
            previousErrorAltitude = errorAltitude;
        } else {
            // if the drone is not armed, set motor speeds to 0
            motor1.write(0.0);
            motor2.write(0.0);
            motor3.write(0.0);
            motor4.write(0.0);
        }

        // wait for 10ms before reading sensors and RC signals again
        wait_ms(10);
    }
}

void emergencyShutdown(PwmOut& motor1, PwmOut& motor2, PwmOut& motor3, PwmOut& motor4) {
    // stop motors
    motor1.write(0.0);
    motor2.write(0.0);
    motor3.write(0.0);
    motor4.write(0.0);

    // pause for 1 second
    wait_ms(1000);

    // restart program
    main();
}
