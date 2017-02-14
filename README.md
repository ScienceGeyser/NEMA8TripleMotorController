# NEMA8TripleMotorController
Control code for custom motor controller circuit work in progress
Circuit uses 3 ATmega8As each controlling an Allegro A4947 motor driver and an AMS AS5047 Encoder using the SPI bus.
Each ATmega8A is connected via I2C to an ATmega32U4 the 32U4 is used to run the user level command set.
The serial port and reset pin of each 8A is connected to the 32U4 so it can act as an in circuit programmer.
the 32U4 also has a ESP8266 connected to the serial port for communication over a network.
