# 🚀 Odyssey

### Thrust Vector Controlled Rocket

> **Odyssey** is an actively stabilized rocket featuring a custom 2-axis Thrust Vector Control (TVC) system with a parachute mechansim , a custom flight computer packed MPU6050 IMU, PID-based control, and onboard flight-data logging, which handles , G-forces, yaw, pitch and PID calulations.
> This also includes a custom launch radio remote Igniton system.

![Odyssey](media/images/rocket.jpg)

## Overview

**Odyssey** explores an alternative to conventional fins rocket stabilization.

It has been a dream of mine to build a guided rocket, for which I explored an alternative method to conventional fins rocket stablization, and Thrust vector control instead. This works by moving the rocket's motor using a custom 2 axis servo gimble, yaw and pitch, relying on correction angles received from the flight computer running a custom PID algrothim. 

Odyssey also has a custom Launch-pad radio controlled Igniton system, which helps ignite the rocket's motor safely using custom e-matches. 

At apogee the rocket's mechnical nose opens with the help of a servo, revealing a parachute that brings the rocket back safely to the ground.

The flight log data is then recorded into a 8 gb SD card, which can be later read and visualised using excel 2d graphs feature. 

### PID Closed-Loop  TVC Control System

                         ┌──────────────────────┐
                         │   Desired Attitude   │
                         │      / Reference     │
                         └──────────┬───────────┘
                                    │
                                    ▼
                              ┌───────────┐
                              │  SUM /    │
                    ┌────────►│   ERROR   │
                    │         │  θref - θ │
                    │         └─────┬─────┘
                    │               │
                    │               ▼
                    │        ┌─────────────┐
                    │        │     PID     │
                    │        │ Controller   │
                    │        │              │
                    │        │ P + I + D   │
                    │        └──────┬──────┘
                    │               │
                    │        TVC command
                    │               │
                    │               ▼
                    │        ┌─────────────┐
                    │        │   Servos +  │
                    │        │  TVC Gimbal │
                    │        └──────┬──────┘
                    │               │
                    │          Thrust Vector
                    │               │
                    │               ▼
                    │        ┌─────────────┐
                    │        │   ROCKET    │
                    │        │   Dynamics  │
                    │        └──────┬──────┘
                    │               │
                    │          Attitude θ
                    │               │
                    │               ▼
                    │        ┌─────────────┐
                    └────────│   MPU6050   │
                             │     IMU     │
                             └─────────────┘


The controller continuously compares the desired Yaw and Pitch with the measured ones. The resulting error is processed by the PID controller, which generates a TVC command. The Output result then send a correction angle for the servos to move in a way that steers the rocket back to the vertical axis, the loop repeats until desired postion is reached.


### Features

     

### Rocket

* Vertical IMU Mounting: Custom math engine tailored for vertical IMU orientation.
* Flight State Machine: Automated transitions through PAD, IGNITION, COAST, APOGEE, and LANDED states.
* Barometric & IMU Telemetry: Live logging of altitude, acceleration, attitude angles, and PID servo outputs directly to an onboard microSD card.
* Fully custom 3d printed design.
* 2-axis TVC gimble.
* Automatic parachute mechanical syetem at apogee.
* Onboard data logging.


### 📡 Wireless Launch System

* Wireless launch command
* 3 step launch verfication.
* RF controlled relay
* Remote ignition control
* Launch-system status indicator
* Physical safety interlock

##Control and Mathematics Overview

1. Attitude Estimation (Vertical Mount)Because the MPU6050 is mounted vertically (aligned with the airframe's longitudinal axis), orientation mapping uses:$$\text{Pitch}_{\text{acc}} = \arctan\left(\frac{-Acc_Y}{\sqrt{Acc_X^2 + Acc_Z^2}}\right) \times 57.296$$$$\text{Yaw}_{\text{acc}} = \arctan\left(\frac{Acc_Z}{\sqrt{Acc_X^2 + Acc_Y^2}}\right) \times 57.296$$Angles are combined via a Complementary Filter ($\alpha = 0.98$):$$\text{Pitch}(t) = 0.98 \cdot (\text{Pitch}_{t-1} + Gyro_Y \cdot dt) + 0.02 \cdot \text{Pitch}_{\text{acc}}$$$$\text{Yaw}(t) = 0.98 \cdot (\text{Yaw}_{t-1} + Gyro_Z \cdot dt) + 0.02 \cdot \text{Yaw}_{\text{acc}}$$2. PID Control LoopGimbal deflection output is calculated to counteract measured angular errors:$$e(t) = \text{Target Angle} - \text{Current Angle}$$$$\text{Output} = K_p \cdot e(t) + K_i \int e(t)\,dt + K_d \cdot \frac{de(t)}{dt}$$Outputs are mechanically constrained to $\pm 15^\circ$ maximum gimbal deflection.

---

## Electronics

| Component      | Purpose             |
| -------------- | ------------------- |
| Arduino Uno    | Flight computer     |
| MPU6050        | Inertial sensing    |
| Servo motors   | TVC actuation       |
| SD card module | Flight-data logging |
| [Battery]      | Power               |

### Architecture

```text
                 MPU6050
                    │
                    │ I²C
                    ▼
              ┌───────────┐
              │  Arduino  │
              │    Uno    │
              └─────┬─────┘
                    │
             ┌──────┴──────┐
             ▼             ▼
        TVC Servos      SD Card
             │
             ▼
        Gimbal motion
```

---

## Data Logging

Odyssey includes an SD-card data logger for recording sensor and flight information.

Example:

```csv
time,altitude,pitch,yaw
0.00,...
0.01,...
0.02,...
0.03,...
```

The recorded data can be used to analyze:

* Attitude
* Angular velocity
* Altitude
* TVC commands
* Controller response

---

## Testing

The system was developed and tested progressively.

### Component Testing

* [x] MPU6050 communication
* [x] Sensor readings
* [x] Arduino flight computer
* [x] Servo control
* [x] TVC gimbal movement
* [x] SD-card logging

### Control Testing

* [x] IMU response to rotation
* [x] PID response
* [x] Closed-loop correction
* [x] TVC response

### Flight Testing

* [ ] Powered flight
* [ ] In-flight stabilization
* [ ] Flight-data validation

---

## Simulation

A conventional rocket simulator such as OpenRocket can be useful for analyzing the airframe and estimating trajectory.

However, **OpenRocket does not simulate Odyssey's custom TVC feedback controller**.

Because Odyssey is designed around active thrust-vector control, a simulation that does not include the controller may show the vehicle becoming unstable or tumbling. This is expected and does not represent the behavior of the closed-loop TVC system.

For this reason, OpenRocket results are treated as **airframe/trajectory analysis rather than TVC validation**.

The TVC system is instead validated through testing of the actual flight computer, IMU, PID controller, servos, and gimbal.

A dedicated TVC simulation is planned as future work.

---

## Current Status

### ✅ Completed

* Rocket CAD
* Rocket construction
* 2-axis TVC mechanism
* Arduino flight computer
* MPU6050 integration
* PID controller
* Servo control
* SD-card logging
* Ground testing

### ⏳ Pending

* Powered flight
* In-flight TVC validation
* Flight-data comparison

The flight test has not yet been performed because a suitable **G-class motor is currently unavailable locally**.

Therefore, **Odyssey is currently a ground-tested prototype and should not be considered flight-proven.**

---

## Repository Structure

```text
Odyssey/
│
├── CAD/
│   ├── rocket/
│   └── TVC/
│
├── electronics/
│   ├── schematic/
│   └── wiring/
│
├── firmware/
│   └── flight_computer/
│
├── simulation/
│
├── testing/
│
├── media/
│   ├── images/
│   └── videos/
│
└── README.md
```

---

## Lessons Learned

Building Odyssey involved several iterations across the mechanical, electrical, and control systems.

Some of the biggest challenges were:

* Servo power requirements
* Mechanical backlash
* IMU axis alignment
* Sensor noise
* PID tuning
* Reliable servo control
* Power-system stability
* Integrating the electronics into a compact flight computer

The project demonstrated that building a control system for a physical vehicle introduces problems that are difficult to see from theory alone.

---

## Future Work

* Conduct the first controlled powered flight
* Validate TVC stabilization in flight
* Analyze recorded flight data
* Tune the controller using flight data
* Develop a dedicated TVC simulation
* Improve attitude estimation
* Characterize actuator response and mechanical backlash

---

## Project Status

**Odyssey — Ground Tested · Flight Pending**

Built for **Hack Club Macondo 2026**.

## License

Software in this repository is released under the **MIT License**.
