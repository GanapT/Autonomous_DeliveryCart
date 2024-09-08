# Autonomous_DeliveryCart

### Description
This project is designed to control a autonomous robot using the FRDM-KL46Z microcontroller. The robot utilizes PWM signals to control the motors and ADC to read light sensor values. The robot can follow a line, move forward, turn, and adjust speed based on sensor input.

### Files Included
- `Controls.c`: The main program file containing all the necessary functions for motor control, ADC setup, and line tracing logic.

### Requirements
- KL46Z Microcontroller
- Two light sensors (connected to ADC channels)
- Motor driver (connected to TPM channels)
- LEDs and buttons for user interaction
- Peripheral library for KL46Z

### Functions

#### Delay
Delays the execution for a specified time.

#### Delay_n
Calls the `delay` function multiple times for extended delay.

#### Ports
Initializes the necessary ports and pins for motor control and sensor input.

#### Motion_Motor
Controls the direction and movement of the motors.

#### Speed
Adjusts the speed of the motors for turning.

#### ADC
Initializes and calibrates the ADC for light sensor input.

#### ADCLeft
Reads the ADC value from the left light sensor.

#### ADCRight
Reads the ADC value from the right light sensor.

#### Motor_5
Sequence of motor movements for a specific path.

#### Motor_S
Controls the motors to follow an 'S' shaped path.

#### TraceLine
Uses ADC values to control the motors for line following.

### Main Program
The `main` function initializes the board, peripherals, and ports, and then enters an infinite loop waiting for a button press to start the line-following logic.

### Usage
1. Compile and upload the program to the KL46Z microcontroller.
2. Ensure that the light sensors are properly connected to the ADC channels.
3. Place the robot on a track with a clear line.
4. Press the start button (connected to PORTC pin 12) to begin line following.

### License
This project is open-source and free to use and modify.

### Acknowledgements
- Thanks to the developers of the peripheral library for KL46Z.
- Inspiration from various line-following robot projects available online.

## Developer
- Name: Ganap Tewary
- Development Version: 0.01
