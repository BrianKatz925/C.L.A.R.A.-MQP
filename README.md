# C.L.A.R.A.-MQP

This is the GitHub repository for the 2021-22 CLARA MQP.

## Instructions
### Prerequisites

1. Complete the prerequisites of the [CLARA-Gamepad Repository](https://github.com/phonymacoroni/CLARA-Gamepad).
2. Connect an ESP32 device (ESP32DevKitC, TinyPICO, etc.) to a USB port on your computer, this will be referred as the external ESP.
4. Upload the [`mac_address.ino` code](TinyPICO_Sender_Code/mac_address) to the external ESP and open the Serial Monitor, write down the MAC address.
5. Locate the [robotcode folder](robotcode), which contains the code required to operate the robot.
6. In the `pico_master.ino` and `inverse_kinematics_pico_master.ino` file, replace the current MAC address with that of the external ESP.
7. Connect the TinyPICO that will be going into the mainboard and retrieve its MAC address by repeating step 3.
8. In the `pico_sender.ino` and `inverse_kinematics_esp_sender.ino` files, replace the current MAC address with that of your TinyPICO.
9. For normal robot operation, upload the `pico_sender.ino code` to the external ESP and the `pico_master.ino` code to the mainboard TinyPICO.
10. For inverse kinematics demonstration, upload the `inverse_kinematics_esp_sender.ino` code to the external ESP and the `inverse_kinematics_pico_master.ino` to the mainboard TinyPICO.

### Operation
#### Typical Robot Operation
Typical Robot Operation requires the use of an external Logitech Gamepad as the form of input.

Refer to the control protocol for the Gamepad below:

| Input  | WiFi Sender Command  | I2C Status Command | Smart Motor Driver Action |
|:------------:|:---------------:|:-----:|:-----:|
| Default | "0" | 0 | Brake all motors | 
| Y Button | "1" | 1 | Rotate lead screw (0x07) - increase diameter |
| X Button | "2" | N/A | Request encoder, current sensor, motor rpm data from all motors |
| A Button | "4" | 2 | Rotate lead screw (0x07) - decrease diameter |
| Right Bumper | "8" | 8 | Drive wheel motors forward - fast speed |
| Left Bumper | "9" | 9 | Drive wheel motors reverse - fast speed |
| DPad 0 (North) | "10" | 0 | Brake all motors - homing |
| DPad 1 (Northeast) | "11" | N/A | Decrease all cable lengths |
| DPad 2 (East) | "12" | Cable 1: 12, <br />Cable 2: 0, <br />Cable 3: 0 | Decrease cable 1 (0x04) length |
| DPad 3 (Southeast) | "13" | Cable 1: 12, <br />Cable 2: 12, <br />Cable 3: 0 | Decrease cable 1 (0x04) and cable 2 (0x05) length |
| DPad 4 (South) | "14" | Cable 1: 0, <br />Cable 2: 12, <br />Cable 3: 0 | Decrease cable 2 (0x05) length |
| DPad 5 (Southwest) | "15" | Cable 1: 0, <br />Cable 2: 12, <br />Cable 3: 12 | Decrease cable 2 (0x05) and cable 3 (0x06) length |
| DPad 6 (West) | "16" | Cable 1: 0, <br />Cable 2: 0, <br />Cable 3: 12 | Decrease cable 3 (0x06) length |
| DPad 7 (Northwest) | "17" | N/A | Increase all cable lengths |
| B Button + D Pad 2 | "18" | Cable 1: 13, <br />Cable 2: 0, <br />Cable 3: 0 | Increase cable 1 (0x04) length |
| B Button + D Pad 4 | "19" | Cable 1: 0, <br />Cable 2: 13, <br />Cable 3: 0 | Increase cable 2 (0x05) length |
| B Button + D Pad 6 | "20" | Cable 1: 0, <br />Cable 2: 0, <br />Cable 3: 13 | Increase cable 3 (0x06) length |
| B Button + Right Bumper | "21" | 10 | Drive wheel motors forward - PID slow speed |
| B Button + Right Bumper | "22" | 11 | Drive wheel motors reverse - PID slow speed|
| A Button + Right Bumper | "23" | 3 | Rotate lead screw (0x07) - decrease diameter until current limit |
| Y  Button + Right Bumper | "24" | 4 | Rotate lead screw (0x07) - increase diameter until current limit|



#### Inverse Kinematics Demonstration
The Gamepad can be disconnected with this demonstration.

Instead the Serial Monitor is used to input values for the configuration parameters **S** (arc length), **theta** (bending angle), and **phi** (rotation about z-axis).

The bounds for each variable are defined in the `inverse_kinematics_esp_sender.ino` file.
