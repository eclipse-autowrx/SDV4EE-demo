# arduino_can_motor_driver
This repo contains the Arduino code to control a motor using [simplefoc](https://docs.simplefoc.com/), with a magnetic SPI sensor (magnetic rotary 2 Click).
The motor is controlled via CAN messages, on each corresponding message a reply will get sent containing the angle.

## Setup
This code was written mostly for an Arduino Uno R4 Minima. For CAN the library "Arduino_CAN" is used, for simplefoc "SimpleFOC".
Additionally to use the sensor the generic library "SPI".

To connect the Arduino with the motor, first place the simplefoc shield on top of the Uno R4. The shield has three outputs to connect the motor.
Additionally connect a power supply when in use.
Then connect a CAN transceiver to the corresponding pins (D4 CANTX, D5 CANRX).
Lastly connect the SPI sensor with the corresponding pins on the Arduino.

## Further Information
There is a macro `DEBUG` to enable debug output each second on serial.
This might impact performance so it is recommended to disable it after development.

There is also a macro `DEBUG2` which on each CAN message rx event is outputting requested/measured sensor data on serial.
To be able to flash again after this code is running, you need to double click reset button on Arduino to get it into bootloader.

### Status LEDs (can_motor_new)
There are three status LEDs available.

#### Startup phase
On startup, all LEDs light up.
Green LED is turned off after some basic initialization is finished; this might be too fast for the human eye to see.
Yellow LED is turned off after motor initialization; if it stays on, check motor driver initialization.
Red LED is turned off after FOC calibration is finished; if it stays on, best check hall sensor/motor attached and then reboot the board.
Now Green LED is turned on again.

#### Runtime phase
During runtime, green LED is blinking in second interval when CAN traffic has been received.
If it does not blink maybe check if there is an active sender on the bus.

The yellow LED lights up, in case within the last second there has been a change in requested value over CAN been received.

There currently is no use for red LED during runtime phase, so it shall be turned off all the time.

### Pinout
```
D0 = LED Red (Motor Status)
D1 = LED Yellow (Received requested value changes)
D2 = LED Green (Received CAN messages)
D3 = B phase PWM pin
D4 = CAN TX to transceiver
D5 = CAN RX from transceiver
D6 = C phase PWM pin
D7 = 
D8 = 
D9 = A phase PWM pin
D10 = SPI SS
D11 = SPI MOSI
D12 = SPI MISO
D13 = SPI SCK
D14 = I2C SDA
D15 = I2C SCL
```
### FOC Shield
used SimpleFOC v2

https://docs.simplefoc.com/arduino_simplefoc_shield_fabrication_v2

Following solder bridges to be used:

PWM A: 3

PWM B: 9

PWM C: 6

enable: 8

phase A: A0 (for current sensing, unused)

phase B: A2 (for current sensing, unused)

range: 5V

## Troubleshooting
### Error Arduino_Can.h not found
Please make sure you selected target `Arduino UNO R4 Minima` and not e.g. the classic `Arduino UNO` or `Arduino UNO Mini` devices.
