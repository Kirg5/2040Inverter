# 2040Inverter
 RP2040 Based BLDC Inverter for sensored motors, capable of putting out 50a 100v, for use in high power ebike/escooter applications.
 
![Inverter PCB](https://github.com/Kirg5/2040Inverter/blob/main/Images/PXL_20230803_063624892.jpg)

## Usage
This is a standalone motor inverter, and not a complete motor controller. It does not contain any inputs for assist sensors or throttles as these are sent to the inverter over UART by a seperate MCU, located in the display. You can find the files for the display [here](https://github.com/Kirg5/RP2040-ILI9488-HUD). 

Upon power-up, the fan will breifly be set to full speed, the brake light will turn white, and 6 tones from 1khz to 6khz will be played as part of the self test sequence. If less than 6 tones play it indicates an issue sending power to each of the motor phases and the inverter will throw itself into an error state. Error states can also be thrown if an issue is detected with any of the sensors (current sensor, voltage sensor, either temperature sensor, or motor encoder)

The inverter shall recieve a throttle input from the MCU in the display over uart with a range of 0-255. These values correlate to the current that the inverter will attempt to draw from the battery, ranging from 0 to the maximum set forward current as defined in the firmware. The maximum forward current should never be set higher than 50A with this inverter (30A without active cooling). The throttle does not directly control duty cycle, and instead the duty cycle is controlled through a PID loop that will match the current draw to the requested current. This PID system will work at low speeds, but as the motor EMF approaches the battery voltage at high speeds, the ability to continue to feed current into the motor will fall off. When the PID loop can no longer provide the requested current to the motor (when Dutycycle<sub>PID</sub> > Dutycycle<sub>throttle</sub>), it will smoothly switch to using the direct value from the throttle instead of the PID controlled value.

This inverter supports regenerative braking, where the kinetic energy that would normally be lost to heat while decellerating gets converted into electrical energy to charge the battery. It works in a very similar fashion to the forward power control where a regen signal with a value ranging from 0-255 is sent from the display MCU to the inverter, and it uses a pid loop to map these values to a range of 0 to the maximum reverse current defined in the firmware. The maximum reverse current should be set to the maximum charge current of your BMS, or the maximum charge current of your cells multipled by the number of parallel groups, whichever is lower. You should never set the maximum reverse current value higher than 50a even if your battery can handle it as the current sensor on the inverter cannot measure currents this high, and it can cause damage to the PCB and it's components, especially if you are not using active cooling.

## Wiring 
Wiring the inverter is fairly straightforward; there are 5 large power pads along the top edge and a 16 pin JST connector along the bottom edge, and 2 sets of labelled through holes for the thermistor and the fan.
### The 5 power pads as ordered from left to right are:
1. +VCC (abs. max 100V)
2. GND
3. Phase A (Blue Phase)
4. Phase B (Green Phase)
5. Phase C (Yellow Phase)

### The 16 pins on the JST connector in numerical order are:
1.  LedR    (For brake light, active low)
2.  LedG    (For brake light, active low)
3.  LedB    (For brake light, active low)
4.  RlyCTL  (Output to main relay MOSFET/BJT)
5.  PWR_OK  (Signal from display MCU that precharging has finished and it is ok to power on)
6.  BOOTSEL (RP2040 bootsel button, for firmware flashing)
7.  +12v    (12v 500mA output)
8.  +5v     (5v 500mA output)
9.  USB_DP  (To USB port)
10. USB_DM  (To USB port)
11. UartTX  (To display)
12. UartRX  (To display)
13. HallC   (Blue hall line, active low)
14. HallB   (Yellow hall line, active low)
15. HallA   (Green hall line, active low)
16. GND
