# pressure_fan_controller

A personal project to create a pressure-based fan speed controller.  The idea is that, in a well airsealed house, we can use a pressure sensor (probably a sensirion sdp810-125pa) to measure the differential pressure between inside and outside, and use that to ramp a makeup air fan up and down, to try to zero out the pressure differential.

Since most controllable HVAC fans use 0-10v DC logic for the control signal, I decided to use a controllable potentiometer (Adafruit DS3502 breakout board) as a simple voltage divider.  An alternative design could use a PWM signal, but that would still need to be amplified to the 0-10v signal range of the fan.

References:
- [Adafruit DS3502 Potentiometer](https://www.adafruit.com/product/4286)
- - [Technical info & Tutorial](https://learn.adafruit.com/ds3502-i2c-potentiometer/)
- [Sensirion SDP810-125pa differential pressure sensor](https://sensirion.com/products/catalog/SDP810-125Pa/)
- - [Arduino library for Sensirion sensor](https://github.com/sensirion/arduino-sdp)
