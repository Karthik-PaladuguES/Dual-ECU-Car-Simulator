# Dual-ECU Car Simulator 🚗

A complete hardware simulation of a vehicle's electronic systems using two **STM32F401RE Nucleo-64** microcontrollers. 

This project divides the vehicle's logic into two separate Electronic Control Units (ECUs) that communicate continuously. ECU #1 handles all driver inputs (steering, pedals, dashboard buttons), packages the data, and transmits it to ECU #2, which drives the physical actuators (motors, servos, lights, and displays).

## 🧠 System Architecture

While standard automotive systems utilize the CAN bus protocol, this prototype implements a custom, high-speed **UART data link (57600 baud)** for reliable point-to-point communication within the Arduino IDE environment. 

### ECU #1: Control System (Sender)
Acts as the driver interface and sensor hub. It reads physical inputs and constructs a 14-byte control packet every 20ms.
* **Steering & Pedals:** 2x PS2 Joysticks for variable steering angle, acceleration, and braking.
* **Dashboard Controls:** Push buttons for gear selection (Fwd/Rev), parking lights, and headlights.
* **Wiper Control:** Potentiometer for variable wiper speed.
* **Safety Systems:** HC-SR04 Ultrasonic sensor for the Automatic Braking System (ABS) and an IR Receiver for remote Lock/Unlock functionality.

### ECU #2: Actuator System (Receiver)
Parses the incoming 14-byte packets and drives the physical vehicle components.
* **Drive Train:** L298N Motor Driver powering 2x DC Motors (Rear wheels) with PWM speed control.
* **Steering & Wipers:** 2x Servo Motors.
* **Instrumentation:** 2x TM1637 7-Segment displays showing calculated Speed (km/h) and RPM.
* **Lighting & Audio:** LED bar graph for parking lights, PWM-controlled brake lights, turn indicators, headlight relay, and dual passive buzzers for horn and indicator sounds.

## 📦 Hardware Required
* 2x STM32F401RE Nucleo-64 Boards
* 1x L298N Motor Driver & 2x DC Motors
* 2x Servo Motors (SG90/MG996R)
* 2x PS2 Joystick Modules
* 2x TM1637 7-Segment Displays
* 1x HC-SR04 Ultrasonic Sensor
* 1x IR Receiver (TSOP1738) & IR Remote
* 1x 12V Relay Module & DC Bulb (Headlight)
* Assorted Push Buttons, Potentiometer, LEDs, and Passive Buzzers

## 🚀 Installation & Setup

1. **Wiring:** * **CRITICAL:** Connect `GND` to `GND` between both boards.
   * Cross-connect the UART link: ECU1 `D8 (PA9 TX)` -> ECU2 `D2 (PA10 RX)` and ECU1 `D2 (PA10 RX)` -> ECU2 `D8 (PA9 TX)`.
   * For the complete physical pinout and wiring schematics, open the `Hardware_Wiring/Connection_Diagram.html` file in any web browser.

2. **Software Dependencies:**
   Install the following via the Arduino IDE Library Manager:
   * `IRremote` (v3.x by shirriff/z3t0)
   * `TM1637Display` (by Avishay Orpaz)
   * *Note: The `Servo.h` library is included natively in the STM32duino core.*

3. **Flashing the Boards:**
   * Ensure you have the [STM32duino core](https://github.com/stm32duino/BoardManagerFiles) installed in your IDE.
   * Select **Nucleo-64** -> **Nucleo F401RE**.
   * Upload `ecu1.ino` to the Control board and `ecu2.ino` to the Actuator board.

## 📡 UART Packet Structure
Communication is handled via a custom 14-byte packet to ensure data integrity:
`[0xAA] [0x55] [B0..B10] [CHK] [0xFF]`
* `B0`: Steering Angle (0-180)
* `B1`: Motor Speed (0-100%)
* `B2`: Direction (0=Stop, 1=Fwd, 2=Rev)
* `B3-B10`: Brakes, Horn, Indicators, Wipers, Lighting, Lock state, and ABS intervention.
* `CHK`: XOR Checksum of the payload.

## 👨‍💻 Author
*B V Raju Institute of Technology (BVRIT)*
