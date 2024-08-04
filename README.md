# TheReckoning
The Reckoning sculpture sensors and input devices.


<p align="center">
  <img src="docs/reckoningPinout" />
  <img src="https://github.com/index01/theReckoning_sensors/blob/main/reckoningPinout.png?raw=true" />
</p>

## Quickstart
Things to know for development, deployment and debugging.

### development
To re-program the device you need a USB-C cable that supports data. Select the Adafruit Feather esp32-S2 NO PSRAM. Make
sure the board configuration is set to partition: default. After that do the usual stuffs. 

### deployment
Power the device with a USB-C power cable providing 5V. 
Connect an ethernet cable to the Reckoning IP Network.
Connect the load cells to the amplifiers. 

## Troubleshooting
Standard operation is indicated by a red heartbeat LED with a one second interval. 
When an OSC message is dispatched on ethernet the green LED will flash. 

### re-calibrating
Press and hold the Tare/Calibrate button until both the Red and Green LEDS rapidly flash 5x. This indicates the load
cells were successfully re-calibrated. Whatever the weight on the load cells is at the time of re-calbration will be the
zero weight. 

### trim potentiometer
Adjust the trim pot with a small screwdrive to change the minimum weight value. All the way to the left will allow all
load cell values through, even if they are negative or way out of bounds. All the way to the right requires the maximum
supported weight, initially configured for 120lbs per load cell. Any potentiometer travel in between 0-100% will be a
percentage of the max wight in lbs.


## Other things you maybe should know
If you receive partition errors chek the above partition info or put the feather in bootload mode by holding down the boot button, pressing the reset button, then releasing the boot button. At that point the lights should all turn off but the serial port will still show up. You should now be able to flash the code, then press reset. This seems to happen periodically, perhaps subsequent bootloader firmwarez will be better.
The IPs and MAC address will need to be updated for the production network.

