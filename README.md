# HTS221-Example
Example project in Atmosphere IoT using nRF52840 DK with HTS221 Temperature and Humidity sensor

![](https://github.com/OldMax44/HTS221-Example/blob/master/images/BoardWiring.JPG)

This project was tested on the above board.  Atmoshpere uses Mbed OS and didn't 
provide any configuration parameters for the HTS221 sensor.  Looking over the generated code,
the HTS221 is set up for 1 Hz continuous sampling and 16 sample temperature averaging and
32 sample humidity averaging.

The hex file downloaded from Atmosphere was easily programmed on the NRF52840 DK via the existing JLink.
You can drag and drop the hex file into the JLink window that pops up when the board is connected to
a USB port and powered on.

The project was set up to read temperature every 5 seconds.  Output via the COM port is shown below.
The temperature and humidity bounced around some, but were correct for the room.

![](https://github.com/OldMax44/HTS221-Example/blob/master/images/Atmosphere_BLE_HTS221.JPG)
