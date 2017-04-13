# Exocoetidae_accelero
This code extracts accelerometer values from an MPU9250 with an ESP8266.

This code is extrated from https://github.com/iotpipe/mpu9250-datalogger-esp8266/blob/master/mpu9250-datalogger-esp8266.ino

This code extracts accelerometer values from an MPU9250 with an ESP8266.
Special thanks to Iotpipe and Kris Winer for his awesome MPU9250 library.

Hardware setup:
VDD ---------------------- 3.3V
SDA ----------------------- Pin4
SCL ----------------------- Pin5
GND ---------------------- GND

EEPROM
Vcc to 3.3V
GND to GND
AO, A1, A2 to GND  (on 24LC256 this gives an i2c slave address as 1010000 which is 0x50)
SDA/SCL to Pin4 and Pin5 of Adafruit Huzzah, respectively
