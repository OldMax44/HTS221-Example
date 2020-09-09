
## HTS221  Connected to Raspberry Pi Zero W

This example is the HTS221 sensor connected to a Raspberry Pi Zero W.
 With the availability of WiFi, I set up an Apache web server.  
 
A basic C program communicates with the HTS221 sensor via the I2C bus using the WiringPi
library.  It reads the calibration coefficients, takes sensor readings, and
runs the interpolation calculation.  Print satements show the calibration coefficients
and the readings for temperature and humidity.  The program just executes a single one shot 
conversion.  Status is checked once a msec after the conversion is launched as a crude
time to convert measure.

It's an old technique, but the C program
writes a simple XML file that is periodcally read by the served web
page.  I also included some radial guages to display the temperature and
humidity.

Note:  The future of the WiringPi library is undertermined now, but it was
included in my Raspbian distribution (Buster) and worked well.

![RPi Zero W Wiring](https://github.com/OldMax44/HTS221-Example/blob/master/images/RPi%20Zero%20W%20Wiring%20to%20HTS221.JPG)

I used Netbeans for the C program development as described at:
https://raspberry-projects.com/pi/programming-in-c/getting-your-raspberry-pi-ready-for-c-programming
This worked extremey well for code development and debugging.

The gauge library can be found at:
https://canvas-gauges.com/

<img src = "https://github.com/OldMax44/HTS221-Example/blob/master/images/HTS221_Gauges.JPG" height="647">

The hts221 file is the compiled executable.  It has to be run as root and writes the data.xml file
out to /var/www/html.  The index.htm file also resides at /var/www/html and is the default web page.

For $10 plus the cost of a microSD card, the RPi Zero W is impressive.  It's not really designed for
battery operation, however.  It would be nice for a temperature and humidity sensor to operate that way
and be mobile and self contained.
