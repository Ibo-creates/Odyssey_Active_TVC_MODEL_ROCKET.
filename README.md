# 🚀 Odyssey

### A Thrust Vector Controlled Rocket

> **Odyssey** is an actively stabilized rocket featuring a custom 2-axis Thrust Vector Control (TVC) system with a parachute mechansim , a custom flight computer packed MPU6050 IMU, PID-based control, and onboard flight-data logging, which handles , G-forces, yaw, pitch and PID calulations.
> This also includes a custom launch radio remote Igniton system.

  
## Overview

**Odyssey** explores an alternative to conventional fins rocket stabilization.

(A flight hasnt be logged yet due to restrictions of commerically available rocket motors in my country)

It has been a dream of mine to build a guided rocket, for which I explored an alternative method to conventional fins rocket stablization, and Thrust vector control instead. This works by moving the rocket's motor using a custom 2 axis servo gimble, yaw and pitch, relying on correction angles received from the flight computer running a custom PID algrothim. 

Odyssey also has a custom Launch-pad radio controlled Igniton system, which helps ignite the rocket's motor safely using custom e-matches. 

At apogee the rocket's mechnical nose opens with the help of a servo, revealing a parachute that brings the rocket back safely to the ground.

The flight log data is then recorded into a 8 gb SD card, which can be later read and visualised using excel 2d graphs feature. 





## Features

     

### Rocket

* Dry mass of approximately: 1kg
* height: 60 cm
* Outer diameter : 9.2 cm
* Rocket motor class to be used : Estes G class motor

* Vertical IMU Mounting: Custom math engine tailored for vertical IMU orientation.
* Flight State Machine: Automated transitions through PAD, IGNITION, COAST, APOGEE, and LANDED states.
* Barometric and IMU Telemetry: Live logging of altitude, acceleration, attitude angles, and PID servo outputs directly to an onboard microsd card.
* Fully custom 3d printed design.
* 2-axis TVC gimble.
* Automatic parachute mechanical syetem at apogee.
* Onboard data logging.


### Wireless Launch System

* Wireless launch command
* 3 step launch verfication.
* Remote ignition control
* Launch system status indicator

### Parameters logged

| Parameter   | Description                   |
| ----------- | ----------------------------- |
| Time_ms     | Timestamp                     |
| State       | Current flight state          |
| Pitch       | Estimated pitch angle         |
| Yaw         | Estimated yaw angle           |
| Altitude    | BMP280 altitude               |
| GForce      | Calculated total acceleration |
| OutputPitch | TVC pitch correction          |
| OutputYaw   | TVC yaw correction            |




The controller continuously compares the desired Yaw and Pitch with the measured ones. The resulting error is processed by the PID controller, which generates a TVC command. The Output result then send a correction angle for the servos to move in a way that steers the rocket back to the vertical axis, the loop repeats until desired postion is reached.

## Flight state machine

PAD
 │
 │ Liftoff detected
 
IGNITION
 │
 │ Burnout detected
 
COAST
 │
 │ Apogee detected
 
APOGEE
 │
 │ Deployment
 
LANDED
## Control and Mathematics Overview


### PID Control math 

Odyssey uses a PD controller to convert attitude error into TVC servo corrections:

$$
u = K_p e + K_i\int e\,dt + K_d\frac{de}{dt}
$$

#### Controller Parameters

| Axis  |  Kp |  Ki |   Kd |
| ----- | --: | --: | ---: |
| Pitch | 1.2 | 0 | 0.05 |
| Yaw   | 2.0 | 0 | 0.10 |

Where:

* e = target attitude − measured attitude
* kp = proportional gain
* ki = integral gain
* kd = derivative gain
* u = TVC correction

The controller runs at **100 Hz**, with the TVC output limited to **±15°**.


---

## Hardware Electronics 

### Flight computer

|  # | Component                   | Quantity |
| -: | --------------------------- | -------: |
|  1 | Arduino Uno                 |        1 |
|  2 | MPU6050 IMU                 |        1 |
|  3 | BMP280 Barometric Sensor    |        1 |
|  4 | MicroSD Card Module         |        1 |
|  5 | Servo Motors                |        3 |
|  6 | Buzzer                      |        1 |
|  7 | NPN Transistor              |        1 |
|  8 | 5mm Green LED               |        1 |
|  9 | Toggle Switch               |        1 |
| 10 | LM7805 5V Voltage Regulator |        2 |
| 11 | 18650 Li-ion Cells          |        2 |
| 12 | 4 × 6 cm Perfboard.         |        1 |

### LaunchPad igniton
|  # | Component                                                | Quantity |
| -: | -------------------------------------------------------- | -------: |
|  1 | Arduino MKR WiFi 1000                                    |        1 |
|  2 | 4-Channel NIC2262/2272 Wireless Remote + Receiver Module |        1 |
|  3 | 5V Relay                                                 |        1 |
|  4 | LM7805 5V Voltage Regulator                              |        1 |
|  5 | Buzzer                                                   |        1 |
|  6 | NPN Transistor                                           |        1 |
|  7 | 5mm LED                                                  |        1 |
|  8 | Diode                                                    |        1 |
|  9 | ON/OFF Switch                                            |        1 |
| 10 | 18650 Li-ion Cells                                       |        3 |
| 11 | 8 × 12 cm Perfboard.                                     |        1 |

## Pinout and Wiring

### Flight computer

| Arduino Uno Pin | Component   | Function             |
| --------------- | ----------- | -------------------- |
| **A4 / SDA**    | MPU6050     | I²C Data             |
| **A5 / SCL**    | MPU6050     | I²C Clock            |
| **A4 / SDA**    | BMP280      | I²C Data             |
| **A5 / SCL**    | BMP280      | I²C Clock            |
| **D10**         | SD Card     | Chip Select          |
| **D11**         | SD Card     | MOSI                 |
| **D12**         | SD Card     | MISO                 |
| **D13**         | SD Card     | SCK                  |
| **D5**          | TVC Servo X | Pitch control        |
| **D6**          | TVC Servo Y | Yaw control          |
| **D9**          | Chute Servo | Parachute deployment |
| **D3**          | Buzzer      | Audio status         |
| **D2**          | Green LED   | Status indicator     |

### Ignition launch pad

| Arduino Pin | Connected To       | Function         |
| ----------- | ------------------ | ---------------- |
| **D3**      | Radio Receiver CH1 | Sequence input 1 |
| **D4**      | Radio Receiver CH2 | Sequence input 2 |
| **D5**      | Radio Receiver CH3 | Sequence input 3 |
| **D6**      | Relay control      | Launch output    |
| **D7**      | Green LED          | Status           |
| **D1**      | Buzzer             | Audio feedback   |


## Software Setup

Required Arduino Libraries
Install the following libraries using the Arduino Library Manager:

1.Adafruit BMP280 Library
2.Servo library 
3.Wire & SPI (Built-in)

## Sd card data logging

### Heres how to read the data from sd card

1. Take out the sd card from the flight computer.
2. Insert it into a laptop or pc using an sd card reader usb.
3. Export the LOG.csv files into excel, google sheets or numbers ( for mac).
4. Make a Seperate column left to the Time_ms, and write to the 2nd row of the new column " = B2/1000 " , drag the yellow box to the end of the column.
5. Filter the time period you want to visualise.
6. Select the two columns you would like to plot on a 2d graph, e.g Altitude at Y axis and Time in seconds in X axis.
7. Click on the " insert " option, select the " Scatter graph with smooth lines " .

                   
---
 

**Odyssey — Ground Tested · Flight Pending**

Built for **Hack Club Macondo 2026**.

## License

Software in this repository is released under the **MIT License**.
