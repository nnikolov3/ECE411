# Project Name: LED Cube (Alias - The Cube)

## Description :

The project is an acrylic cube containing addressable LEDs at the bottom. The LED change their brightness and color in response to the motion of the Cube on the x y z plane. When left static (no movement), it serves as a source of ambient light (night light, desk light etc). Removing the device from the charging station, it turns ON and tilting the device at an angle of more than 45 degrees while static it turns OFF. Therefore, it can either serve as an entertainment device or an ambient light source. The Cube is suitable for all ages, and it is an excellent choice for the entire family.

## The sensor :
The LED Cube incorporates one sensor at the stage of the initial development. Further, it uses a gyroscope sensor to modulate the mode of operation for the LEDs. Gyroscope sensors are devices that can detect angular velocity. A sensor of this type can detect rotational motion and changes in orientation. The specific gyroscope sensor is listed below:
MPU-6050
https://www.digikey.com/product-detail/en/tdk-invensense/MPU-6050/1428-1007-1-ND/4038010

The controller :
The LED Cube is using the ATmega328p microcontroller. 

The high-performance Microchip 8-bit AVR RISC-based microcontroller combines 32KB ISP flash memory with read-while-write capabilities. Besides, it has1KB EEPROM, 2KB SRAM, 23 general-purpose I/O lines, 32 general purpose working registers, three flexible timer/counters with compare modes, internal and external interrupts,serial programmable USART, a byte-oriented 2-wire serial interface, SPI serial port, 6-channel 10-bit A/D converter (8-channels in TQFP and QFN/MLF packages), programmable watchdog timer with internal oscillator, and five software selectable power saving modes. The device operates between 1.8-5.5 volts. (Source: https://www.microchip.com/wwwproducts/en/ATmega328)
What is more, the ATmega328p processor is in the Arduino board; therefore, prototyping and programming are faster and more accessible at the initial stage of the project.


## The actuator:
The actuators in the project are addressable LEDs. The LEDs are individually programmable and change color based on different sets of inputs. The selected LEDs are:
WS2812B1M60LB30
https://www.amazon.com/gp/product/B01CDTED80/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1

