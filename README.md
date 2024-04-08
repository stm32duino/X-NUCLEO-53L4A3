# X-NUCLEO-53L4A3

Arduino library to support the X-NUCLEO-53L4A3 based on VL53L4ED ranging sensor.
This sensor uses I2C to communicate. An I2C instance is required to access to the sensor.
The APIs provide simple distance measure in both polling and interrupt modes.

## Examples

There are 3 examples with the X-NUCLEO-53L4A3 library.

* X_NUCLEO_53L4A3_HelloWorld: This example code is to show how to get proximity values of the onboard VL53L4ED sensor in polling mode.

* X_NUCLEO_53L4A3_HelloWorld_Interrupt: This example code is to show how to get proximity values of the onboard VL53L4ED sensor in interrupt mode.

* X_NUCLEO_53L4A3_Threshold_Detection: This example code is to show how to configure the threshold and get proximity values when the ranging is below 100mm or above 200mm.

## Dependencies

This package requires the following Arduino libraries:

* STM32duino VL53L4ED: https://github.com/stm32duino/VL53L4ED


## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-53L4A3

The VL53L4ED datasheet is available at  
https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l4ed.html