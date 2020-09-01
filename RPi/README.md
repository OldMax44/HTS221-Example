
**************** UNDER CONSTRUCTION *********************************

This example is the HTS221 sensor connected to a Raspberry Pi Zero W.
With the availability of WiFi, I set up an Apache web server.  A C program
communicates with the HTS221 sensor via the I2C bus using the WiringPi
library.  It reads the claibration coefficients, takes sensor readings, and
runs the interpolation calculation.  It's an old technique, but the C program
then writes a simple XML file that is periodcally read by the served web
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

