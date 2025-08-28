# arduino_can_display_driver
 This repo contains the arduino code for Arrduino UNO R4 Wifi
  - LED motor torque CAN-signal visualization
  - OLED CAN-signal visualization for SSD1306 or SSD1305
  - CAN roundtrip Pong for 

## Setup
This code was written for an Arduino Uno R4 Wifi. For CAN the library "Arduino_CAN" is used, for I2C OLED ssd1306 use "Adafruit_SSD1306" or "Adafruit_SSD1305" for ssd1305. 

- ssd1305_can_display_driver.ino use for ssd1305 OLED
- ssd1306_can_display_driver.ino use for ssd1306 OLED

Connect the Arduino UNO R4 with I2C OLED with the corresponding pins on the Arduino SCL, SDA +5V, GND.
Connect a CAN transceiver to the corresponding pins (D10 CANTX, D13 CANRX, +3.3V, GND).
cd ..
Finally connect a power supply.
